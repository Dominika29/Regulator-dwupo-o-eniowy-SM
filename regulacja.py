# regulacja.py
import json
import argparse
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

DATA_DIR = Path("dane")
OUT_DIR = Path("outputs")


@dataclass
class FOPDT:
    K: float        # degC per %
    T: float        # s
    L: float        # s
    y0: float       # degC
    u0: float       # %
    u1: float       # %
    t_step: float   # s
    t_fall: float   # s


# ----------------------------
# Helpers: parsing & cleaning
# ----------------------------
def _to_num_series(s: pd.Series) -> pd.Series:
    s = s.astype(str).str.replace(",", ".", regex=False)
    return pd.to_numeric(s, errors="coerce")


def _ffill_numeric(df: pd.DataFrame, cols: List[str]) -> pd.DataFrame:
    for c in cols:
        if c in df.columns:
            df[c] = _to_num_series(df[c])
            df[c] = df[c].ffill()
    return df


# ----------------------------
# Error metrics for control quality
# ----------------------------
def add_error_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    if "SP_C" not in df.columns:
        return df

    df["t_s"] = _to_num_series(df["t_s"])
    df["T_C"] = _to_num_series(df["T_C"])
    df["SP_C"] = _to_num_series(df["SP_C"])
    df = df.dropna(subset=["t_s", "T_C", "SP_C"]).sort_values("t_s").reset_index(drop=True)

    df["t_s"] = df["t_s"] - float(df["t_s"].iloc[0])

    df["e_C"] = df["SP_C"] - df["T_C"]
    df["abs_e_C"] = df["e_C"].abs()
    df["e2_C2"] = df["e_C"] ** 2
    return df


def integrate_metrics(df: pd.DataFrame, tol_C: float = 0.05) -> dict:
    if not {"t_s", "e_C", "abs_e_C", "e2_C2"}.issubset(df.columns):
        return {"note": "Brak kolumn uchybu (SP_C lub e_C)."}

    t = df["t_s"].to_numpy(dtype=float)
    e = df["e_C"].to_numpy(dtype=float)
    ae = df["abs_e_C"].to_numpy(dtype=float)
    e2 = df["e2_C2"].to_numpy(dtype=float)

    dt = np.diff(t, prepend=t[0])
    if len(dt) > 5:
        dt[0] = float(np.median(dt[1:6]))
    dt = np.clip(dt, 0.0, np.inf)

    IAE = float(np.sum(ae * dt))
    ISE = float(np.sum(e2 * dt))
    ITAE = float(np.sum((t * ae) * dt))

    n = len(df)
    tail = max(5, int(0.1 * n))
    sse = float(np.mean(e[-tail:]))

    in_band = float(np.mean(np.abs(e) <= tol_C)) * 100.0

    return {
        "IAE": IAE,
        "ISE": ISE,
        "ITAE": ITAE,
        "SSE_end_mean_C": sse,
        "in_band_pct": in_band,
        "tol_C": float(tol_C),
        "n_samples": int(n),
        "t_end_s": float(t[-1]) if n else 0.0,
    }


def metrics_by_phase(df: pd.DataFrame, tol_C: float = 0.05) -> dict:
    if "phase" not in df.columns:
        return {}

    out = {}
    ph = df["phase"].astype(str).fillna("")
    for phase_name in sorted(set(ph.values)):
        if not phase_name or phase_name == "nan":
            continue
        dfi = df.loc[ph == phase_name].copy()
        if len(dfi) < 5:
            continue
        out[phase_name] = integrate_metrics(dfi, tol_C=tol_C)
    return out


def detect_sp_steps(df: pd.DataFrame, step_thr_C: float = 0.3) -> list:
    if "SP_C" not in df.columns:
        return []

    sp = df["SP_C"].to_numpy(dtype=float)
    t = df["t_s"].to_numpy(dtype=float)
    dsp = np.diff(sp)

    steps = []
    for k in np.where(np.abs(dsp) >= step_thr_C)[0]:
        steps.append({"k": int(k), "t": float(t[k + 1]), "sp0": float(sp[k]), "sp1": float(sp[k + 1])})
    return steps


def step_response_metrics(df: pd.DataFrame, step: dict, tol_C: float = 0.05, window_s: float = 600.0) -> dict:
    t0 = step["t"]
    sp1 = step["sp1"]

    w = (df["t_s"] >= t0) & (df["t_s"] <= (t0 + window_s))
    d = df.loc[w].copy()
    if len(d) < 10:
        return {"note": "Za mało próbek po skoku."}

    t = d["t_s"].to_numpy(dtype=float)
    y = d["T_C"].to_numpy(dtype=float)
    e = (sp1 - y)

    if step["sp1"] > step["sp0"]:
        peak = float(np.max(y))
        overshoot_C = peak - sp1
    else:
        trough = float(np.min(y))
        overshoot_C = sp1 - trough

    band = np.abs(e) <= tol_C
    hold_s = 30.0

    dt = np.diff(t, prepend=t[0])
    if len(dt) > 5:
        dt[0] = float(np.median(dt[1:6]))
    dt = np.clip(dt, 1e-6, np.inf)
    hold_n = max(1, int(round(hold_s / float(np.median(dt)))))

    t_settle = None
    b = band.astype(int)
    for i in range(0, len(b) - hold_n):
        if np.all(b[i:i + hold_n] == 1):
            t_settle = float(t[i] - t0)
            break

    tail = max(5, int(0.1 * len(d)))
    sse = float(np.mean(e[-tail:]))

    return {
        "t_step_s": float(t0),
        "sp0_C": float(step["sp0"]),
        "sp1_C": float(step["sp1"]),
        "overshoot_C": float(overshoot_C),
        "t_settle_s": float(t_settle) if t_settle is not None else None,
        "tol_C": float(tol_C),
        "SSE_end_mean_C": float(sse),
        "window_s": float(window_s),
    }


# ----------------------------
# Model & identification
# ----------------------------
def fopdt_step_model(t, K, T, L, y0, du):
    t = np.asarray(t, dtype=float)
    y = np.full_like(t, y0, dtype=float)
    m = t >= L
    y[m] = y0 + (K * du) * (1.0 - np.exp(-(t[m] - L) / max(T, 1e-9)))
    return y


def _find_heat_segment(df: pd.DataFrame) -> Tuple[int, int, float, float, float, float]:
    out = df["OUT_pct"].to_numpy(dtype=float)
    t = df["t_s"].to_numpy(dtype=float)

    if len(out) < 20:
        raise RuntimeError("Za mało próbek w logu.")

    dout = np.diff(out)
    jump_thr = max(5.0, 0.2 * np.nanmax(np.abs(dout)))

    k_step = None
    if "phase" in df.columns:
        ph = df["phase"].astype(str).fillna("")
        heat_idxs = np.where(ph.values == "HEAT100")[0]
        if len(heat_idxs) > 0:
            anchor = int(heat_idxs[0])
            lo = max(0, anchor - 20)
            hi = min(len(dout), anchor + 20)
            local = dout[lo:hi]
            if len(local) > 0:
                kk = lo + int(np.argmax(local))
                if dout[kk] > jump_thr:
                    k_step = kk

    if k_step is None:
        kk = int(np.argmax(dout))
        if dout[kk] < jump_thr:
            raise RuntimeError(
                "Nie wykryto wyraźnego skoku OUT w górę. "
                "Upewnij się, że w teście było duże OUT (np. 0->100 albo duży skok)."
            )
        k_step = kk

    t_step = float(t[k_step + 1])
    u0 = float(out[k_step])
    u1 = float(out[k_step + 1])

    k_fall = None
    for i in range(k_step + 1, len(dout)):
        if dout[i] < -jump_thr:
            k_fall = i
            break
    if k_fall is None:
        k_fall = len(out) - 2

    t_fall = float(t[k_fall + 1])
    return int(k_step), int(k_fall), t_step, t_fall, u0, u1


def _fit_quality(y_true: np.ndarray, y_pred: np.ndarray) -> Dict[str, float]:
    y_true = np.asarray(y_true, dtype=float)
    y_pred = np.asarray(y_pred, dtype=float)
    err = y_true - y_pred
    rmse = float(np.sqrt(np.mean(err ** 2)))
    mae = float(np.mean(np.abs(err)))
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else float("nan")
    return {"RMSE_C": rmse, "MAE_C": mae, "R2": r2}


def identify_fopdt(df: pd.DataFrame) -> Tuple[FOPDT, Dict[str, Any]]:
    df = df.copy()

    for col in ("t_s", "T_C", "OUT_pct"):
        if col not in df.columns:
            raise RuntimeError(f"CSV nie ma kolumny {col}.")

    df["t_s"] = _to_num_series(df["t_s"])
    df["T_C"] = _to_num_series(df["T_C"])
    df["OUT_pct"] = _to_num_series(df["OUT_pct"])

    df = df.dropna(subset=["t_s", "T_C", "OUT_pct"]).reset_index(drop=True)
    if len(df) < 30:
        raise RuntimeError("Za mało próbek do identyfikacji (min. ~30).")

    df = df.sort_values("t_s").reset_index(drop=True)
    df["t_s"] = df["t_s"] - float(df["t_s"].iloc[0])

    k_step, k_fall, t_step, t_fall, u0, u1 = _find_heat_segment(df)
    du = (u1 - u0)
    if abs(du) < 5.0:
        raise RuntimeError(f"Skok OUT zbyt mały (du={du:.2f}). Do identyfikacji zrób większy skok.")

    t = df["t_s"].to_numpy(dtype=float)
    y = df["T_C"].to_numpy(dtype=float)

    pre0 = max(0, k_step - 40)
    if (k_step - pre0) >= 8:
        y0 = float(np.median(y[pre0:k_step]))
    else:
        y0 = float(y[k_step])

    post_mask = (t >= t_step) & (t < t_fall)
    t_fit = t[post_mask] - t_step
    y_fit = y[post_mask]

    if len(t_fit) < 15:
        raise RuntimeError("Za mało próbek w segmencie grzania do dopasowania.")

    tail_n = min(25, len(y_fit))
    y_end = float(np.median(y_fit[-tail_n:]))

    K0 = (y_end - y0) / du
    T0 = max(10.0, 0.3 * float(t_fit[-1]))
    L0 = max(0.0, 1.0)

    def model(tf, K, Tpar, Lpar, y0p):
        tf = np.asarray(tf, dtype=float)
        outm = np.full_like(tf, y0p, dtype=float)
        m = tf >= Lpar
        outm[m] = y0p + (K * du) * (1.0 - np.exp(-(tf[m] - Lpar) / max(Tpar, 1e-9)))
        return outm

    bounds = (
        [-200.0,   1.0,   0.0,   y0 - 50.0],
        [ 200.0, 8000.0, 600.0,  y0 + 50.0],
    )

    popt, _pcov = curve_fit(
        model,
        t_fit,
        y_fit,
        p0=[K0, T0, L0, y0],
        bounds=bounds,
        maxfev=30000,
    )

    K, Tpar, Lpar, y0_fit = map(float, popt)

    yhat = model(t_fit, K, Tpar, Lpar, y0_fit)
    q = _fit_quality(y_fit, yhat)

    f = FOPDT(K=K, T=Tpar, L=Lpar, y0=y0_fit, u0=u0, u1=u1, t_step=t_step, t_fall=t_fall)
    meta = {
        "segment": {
            "k_step": int(k_step),
            "k_fall": int(k_fall),
            "t_step_s": float(t_step),
            "t_fall_s": float(t_fall),
            "u0_pct": float(u0),
            "u1_pct": float(u1),
            "du_pct": float(du),
            "y0_est_C": float(y0),
            "y_end_est_C": float(y_end),
            "n_fit_samples": int(len(t_fit)),
            "t_fit_span_s": float(t_fit[-1] - t_fit[0]) if len(t_fit) > 1 else 0.0,
        },
        "fit_quality_heat_segment": q,
    }
    return f, meta


# ----------------------------
# Tuning: IMC PI and IMC PID (with filtered D)
# ----------------------------
def imc_pi_from_fopdt(f: FOPDT, lam: Optional[float] = None):
    K, T, L = f.K, f.T, f.L

    if lam is None:
        lam = max(3.0 * L, 0.6 * T)

    if abs(K) < 1e-12:
        raise RuntimeError("K ~ 0, nie da się policzyć regulatora.")

    Kp = T / (K * (lam + L))
    Ti = T + 0.5 * L
    Ki = Kp / Ti
    Kd = 0.0
    return float(Kp), float(Ki), float(Kd), float(lam), float(Ti)


def imc_pid_from_fopdt(f: FOPDT, lam: Optional[float] = None, dlam: Optional[float] = None, N: float = 10.0):
    """
    IMC PID (przybliżenie) dla FOPDT.
    - lam: agresywność toru P/I
    - dlam: agresywność toru D
    - N: filtr pochodnej
    """
    K, T, L = f.K, f.T, f.L

    if lam is None:
        lam = max(L, 0.2 * T)

    if dlam is None:
        dlam = max(lam, 0.5 * T)

    if abs(K) < 1e-12:
        raise RuntimeError("K ~ 0, nie da się policzyć regulatora.")

    Kp = T / (K * (lam + L))

    Ti = T + 0.5 * L
    Ki = Kp / Ti
    Td = 0.5 * L

    Kd = Kp * Td

    return float(Kp), float(Ki), float(Kd), float(lam), float(Ti), float(Td), float(N), float(dlam)


# ----------------------------
# Main suggestion pipeline
# ----------------------------
def list_csv_files(data_dir: Path):
    if not data_dir.exists():
        return []
    files = sorted(data_dir.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return files


def pick_csv_interactive(files):
    print("\nDostępne logi w ./dane/:")
    for i, p in enumerate(files, 1):
        print(f"{i:2d}) {p.name}")
    while True:
        s = input("Wybierz numer pliku: ").strip()
        try:
            k = int(s)
            if 1 <= k <= len(files):
                return files[k - 1]
        except ValueError:
            pass
        print("Błędny wybór. Spróbuj ponownie.")


def suggest_from_csv(
    csv_path: str,
    out_json: str = None,
    make_plot: bool = False,
    lam: Optional[float] = None,
    dlam: Optional[float] = None,
    N: float = 10.0,
    tol_C: float = 0.05,
) -> Dict[str, Any]:
    df = pd.read_csv(csv_path)
    df = _ffill_numeric(df, ["KP", "KI", "KD"])

    needed = {"t_s", "T_C", "OUT_pct"}
    if not needed.issubset(df.columns):
        raise RuntimeError(f"CSV musi mieć kolumny: {sorted(needed)}. Masz: {list(df.columns)}")

    f, fmeta = identify_fopdt(df)

    # PI
    Kp_pi, Ki_pi, Kd_pi, lam_used, Ti_pi = imc_pi_from_fopdt(f, lam=lam)

    # PID
    Kp_pid, Ki_pid, Kd_pid, lam_pid, Ti_pid, Td_pid, N_used, dlam_used = imc_pid_from_fopdt(
        f, lam=lam_used, dlam=dlam, N=N
    )

    result: Dict[str, Any] = {
        "source_csv": csv_path,
        "fopdt": {
            "K_degC_per_pct": f.K,
            "T_s": f.T,
            "L_s": f.L,
            "y0_C": f.y0,
            "t_step_s": f.t_step,
            "t_fall_s": f.t_fall,
            "u0_pct": f.u0,
            "u1_pct": f.u1,
        },
        "model_state": fmeta,
        "imc": {
            "lambda_used_s": lam_used,
            "d_lambda_used_s": dlam_used,
            "derivative_filter_N": N_used,
        },
        "tuning": {
            "PI": {"Kp": Kp_pi, "Ki": Ki_pi, "Kd": 0.0, "Ti_s": Ti_pi},
            "PID": {"Kp": Kp_pid, "Ki": Ki_pid, "Kd": Kd_pid, "Ti_s": Ti_pid, "Td_s": Td_pid},
        },
        "stm_commands": {
            "PI": [
                f"KP {Kp_pi:.4f}",
                f"KI {Ki_pi:.4f}",
                f"KD {0.0:.4f}",
                "MODE PID",
            ],
            "PID": [
                f"KP {Kp_pid:.4f}",
                f"KI {Ki_pid:.4f}",
                f"KD {Kd_pid:.4f}",
                "MODE PID",
            ],
        },
    }

    # --- quality of actual control from SP tracking (if SP exists) ---
    df_err = add_error_columns(df)
    report = integrate_metrics(df_err, tol_C=tol_C)
    report_phase = metrics_by_phase(df_err, tol_C=tol_C)
    steps = detect_sp_steps(df_err, step_thr_C=0.3)
    step_reports = [step_response_metrics(df_err, st, tol_C=tol_C, window_s=600.0) for st in steps]

    result["quality"] = report
    if report_phase:
        result["quality_by_phase"] = report_phase
    if step_reports:
        result["step_metrics"] = step_reports

    # Write json
    if out_json:
        with open(out_json, "w", encoding="utf-8") as fjson:
            json.dump(result, fjson, indent=2)

    # Save enriched csv
    try:
        OUT_DIR.mkdir(exist_ok=True)
        df_en = df_err.copy()
        enriched_csv = OUT_DIR / f"{Path(csv_path).stem}_enriched.csv"
        df_en.to_csv(enriched_csv, index=False)
        result["enriched_csv"] = str(enriched_csv)
    except Exception:
        pass

    # Plots
    if make_plot and plt is not None:
        df2 = df.copy()
        df2["t_s"] = _to_num_series(df2["t_s"])
        df2["T_C"] = _to_num_series(df2["T_C"])
        df2["OUT_pct"] = _to_num_series(df2["OUT_pct"])
        df2 = df2.dropna(subset=["t_s", "T_C", "OUT_pct"]).sort_values("t_s").reset_index(drop=True)
        df2["t_s"] = df2["t_s"] - float(df2["t_s"].iloc[0])

        t = df2["t_s"].to_numpy(dtype=float)
        y = df2["T_C"].to_numpy(dtype=float)
        out = df2["OUT_pct"].to_numpy(dtype=float)

        post = (t >= f.t_step) & (t < f.t_fall)
        tf = t[post] - f.t_step
        du = (f.u1 - f.u0)
        yhat = fopdt_step_model(tf, f.K, f.T, f.L, f.y0, du)

        plt.figure()
        plt.plot(t, y, label="T measured")
        plt.axvline(f.t_step, linestyle="--", label="step up")
        plt.axvline(f.t_fall, linestyle="--", label="step down")
        plt.plot(t[post], yhat, label="FOPDT fit (heat segment)")
        plt.xlabel("t [s]")
        plt.ylabel("T [°C]")
        plt.grid(True)
        plt.legend()

        plt.figure()
        plt.plot(t, out, label="OUT [%]")
        plt.axvline(f.t_step, linestyle="--", label="step up")
        plt.axvline(f.t_fall, linestyle="--", label="step down")
        plt.grid(True)
        plt.legend()

        if "SP_C" in df2.columns:
            df2e = add_error_columns(df2)
            if "e_C" in df2e.columns:
                plt.figure()
                plt.plot(df2e["t_s"].to_numpy(dtype=float), df2e["e_C"].to_numpy(dtype=float), label="e = SP - T [°C]")
                plt.axhline(0.0, linestyle="--")
                plt.grid(True)
                plt.legend()
                plt.xlabel("t [s]")
                plt.ylabel("e [°C]")

        plt.show()

    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", help="CSV do analizy (opcjonalnie). Jeśli brak, użyj --pick lub --latest.")
    ap.add_argument("--pick", action="store_true", help="Wybierz plik CSV z folderu ./dane/")
    ap.add_argument("--latest", action="store_true", help="Użyj najnowszego CSV z folderu ./dane/")
    ap.add_argument("--out", default=None, help="Gdzie zapisać json z wynikiem (domyślnie outputs/<stem>_pid.json)")
    ap.add_argument("--plot", action="store_true", help="Pokaż wykres dopasowania")
    ap.add_argument("--lam", type=float, default=None, help="Lambda IMC [s] (mniejsze = bardziej agresywnie)")
    ap.add_argument("--dlam", type=float, default=None, help="Lambda dla toru D [s] (zwykle >= lam)")
    ap.add_argument("--N", type=float, default=10.0, help="Filtr pochodnej N (do raportu/zalecen)")
    ap.add_argument("--tol", type=float, default=0.05, help="Tolerancja do metryk (pasmo) [°C]")
    ap.add_argument("--show-pi", action="store_true", help="Wypisz komendy STM dla PI")
    ap.add_argument("--show-pid", action="store_true", help="Wypisz komendy STM dla PID")
    args = ap.parse_args()

    DATA_DIR.mkdir(exist_ok=True)
    OUT_DIR.mkdir(exist_ok=True)

    if args.csv:
        csv_path = Path(args.csv)
    elif args.latest or args.pick:
        files = list_csv_files(DATA_DIR)
        if not files:
            raise SystemExit("Brak plików CSV w ./dane/. Najpierw zrób test w GUI.")
        csv_path = files[0] if args.latest else pick_csv_interactive(files)
    else:
        raise SystemExit("Podaj plik CSV albo użyj --pick / --latest.")

    if not csv_path.exists():
        raise SystemExit(f"Nie znaleziono pliku: {csv_path}")

    if args.out is None:
        out_json = OUT_DIR / f"{csv_path.stem}_pid.json"
    else:
        out_json = Path(args.out)
        out_json.parent.mkdir(parents=True, exist_ok=True)

    res = suggest_from_csv(
        str(csv_path),
        out_json=str(out_json),
        make_plot=args.plot,
        lam=args.lam,
        dlam=args.dlam,
        N=args.N,
        tol_C=args.tol,
    )

    print("\n--- FOPDT (heat segment) ---")
    f = res["fopdt"]
    print(f"K={f['K_degC_per_pct']:.6f} [°C/%]  T={f['T_s']:.3f}s  L={f['L_s']:.3f}s  y0={f['y0_C']:.3f}°C")
    seg = res["model_state"]["segment"]
    fitq = res["model_state"]["fit_quality_heat_segment"]
    print(f"Segment: t_step={seg['t_step_s']:.3f}s  t_fall={seg['t_fall_s']:.3f}s  du={seg['du_pct']:.2f}%  n_fit={seg['n_fit_samples']}")
    print(f"Fit quality (heat): RMSE={fitq['RMSE_C']:.4f}°C  MAE={fitq['MAE_C']:.4f}°C  R2={fitq['R2']:.4f}")

    print("\n--- Suggested tuning ---")
    pi = res["tuning"]["PI"]
    pid = res["tuning"]["PID"]
    print(f"PI : Kp={pi['Kp']:.4f}  Ki={pi['Ki']:.6f}  Kd={pi['Kd']:.4f}  Ti={pi['Ti_s']:.3f}s")
    print(f"PID: Kp={pid['Kp']:.4f}  Ki={pid['Ki']:.6f}  Kd={pid['Kd']:.4f}  Ti={pid['Ti_s']:.3f}s  Td={pid['Td_s']:.3f}s")

    print("\nIMC settings:")
    print(f"lambda={res['imc']['lambda_used_s']:.3f}s  d_lambda={res['imc']['d_lambda_used_s']:.3f}s  N={res['imc']['derivative_filter_N']:.2f}")

    if args.show_pi:
        print("\nSTM commands (PI):")
        for c in res["stm_commands"]["PI"]:
            print(c)

    if args.show_pid:
        print("\nSTM commands (PID):")
        for c in res["stm_commands"]["PID"]:
            print(c)

    print(f"\nSaved: {out_json}")
    if "enriched_csv" in res:
        print(f"Enriched CSV: {res['enriched_csv']}")

    if "quality" in res and "note" not in res["quality"]:
        q = res["quality"]
        print("\n--- Control quality (tracking) ---")
        print(f"IAE={q['IAE']:.3f}  ISE={q['ISE']:.3f}  ITAE={q['ITAE']:.3f}")
        print(f"SSE_end_mean={q['SSE_end_mean_C']:.4f}°C  in_band={q['in_band_pct']:.2f}%  tol={q['tol_C']:.3f}°C")


if __name__ == "__main__":
    main()
