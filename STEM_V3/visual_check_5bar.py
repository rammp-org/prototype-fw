#!/usr/bin/env python3
"""Compute and visualize the STEM 5-bar linkage from the inverse kinematics.

This script does not hardcode the joints. It takes the output point M3 = (x, y)
from the design geometry, solves the IK with the crossed-link branch condition,
and then draws the linkage.

Usage examples:
    python3 visual_check_5bar.py --x 66 --y 138.5
    python3 visual_check_5bar.py --pose full_down
    python3 visual_check_5bar.py --all --save geometry_check.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

M1 = (0.0, 0.0)
M2 = (170.0, 0.0)
J4 = (-38.0, 8.0)

# Documented edge poses from Geometry.md (analysis frame; M3 is the output point)
POSES = {
    "home": {"plate": (66, 360), "M3": (66.00, 208.50)},
    "full_down": {"plate": (66, 290), "M3": (66.00, 138.50)},
    "full_up": {"plate": (66, 720), "M3": (66.00, 568.50)},
    "max_forward": {"plate": (266, 360), "M3": (266.00, 208.50)},
    "max_rearward": {"plate": (-34, 360), "M3": (-34.00, 208.50)},
}

THETA1_FULL_DOWN = math.radians(3.95)
THETA2_FULL_DOWN = math.radians(178.06)
FULL_DOWN_M3 = (66.00, 138.50)
FULL_DOWN_J2 = (309.26, 21.36)

BAR_LENGTH_M3_TO_J2 = 270.0
BAR_LENGTH_M3_TO_J3 = 100.0
BAR_LENGTH_M3_TO_J5 = 200.0
BAR_LENGTH_J1_TO_J3 = 270.0
BAR_LENGTH_J5_PERP_OFFSET = 12.0
CRANK_LENGTH_M1_TO_J2 = 310.0
CRANK_LENGTH_M2_TO_J1 = 270.0


def normalize_angle_deg(angle_deg: float) -> float:
    """Wrap to [-180, 180)."""
    angle = angle_deg % 360.0
    if angle >= 180.0:
        angle -= 360.0
    return angle


def cross2d(a, b, c):
    """2D cross product of vectors (b-a) and (c-a)."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def solve_ik_for_m3(
    x_rel: float,
    y_rel: float,
    bar_length_m3_to_j2: float = BAR_LENGTH_M3_TO_J2,
    bar_length_m3_to_j3: float = BAR_LENGTH_M3_TO_J3,
    bar_length_m3_to_j5: float = BAR_LENGTH_M3_TO_J5,
    bar_length_j1_to_j3: float = BAR_LENGTH_J1_TO_J3,
    bar_length_j5_perp_offset: float = BAR_LENGTH_J5_PERP_OFFSET,
    crank_length_m1_to_j2: float = CRANK_LENGTH_M1_TO_J2,
    crank_length_m2_to_j1: float = CRANK_LENGTH_M2_TO_J1,
):
    """Solve the STEM 5-bar IK for a target M3 point in the M1-origin frame.

    Args:
      x_rel, y_rel: target M3 coordinates in the M1-origin analysis frame.
    bar_length_m3_to_j2: purple-bar length from M3 to J2.
    bar_length_m3_to_j3: purple-bar station from M3 to J3.
    bar_length_m3_to_j5: purple-bar station from M3 to J5.
    bar_length_j1_to_j3: green coupler length from J1 to J3.
    bar_length_j5_perp_offset: J5 offset perpendicular to the purple bar.
    crank_length_m1_to_j2: red crank length from M1 to J2.
    crank_length_m2_to_j1: yellow crank length from M2 to J1.

    Returns a dictionary with:
      M3, J1, J2, J3, J5, theta1_deg, theta2_deg,
      phi1_deg, phi2_deg, plate_angle_deg, m3_angle_deg
    """
    M3 = (float(x_rel), float(y_rel))
    r = math.hypot(M3[0], M3[1])
    if r < 1e-9:
        raise ValueError("M3 is too close to the origin; IK is singular.")

    L1 = float(crank_length_m1_to_j2)
    L4 = float(bar_length_m3_to_j2)
    L4_J3 = float(bar_length_m3_to_j3)
    L4_J5 = float(bar_length_m3_to_j5)
    L3 = float(bar_length_j1_to_j3)
    L2 = float(crank_length_m2_to_j1)

    # Correct geometry: |M3 + L4*u - M1| = L1, with M1 at the origin.
    # This gives: p·u = (L1^2 - L4^2 - |p|^2) / (2*L4)
    # and since p = r * [cos(alpha), sin(alpha)], u = [cos(alpha ± beta), sin(alpha ± beta)]
    # with cos(beta) = (p·u) / r.
    dot_p_u = (L1**2 - L4**2 - r**2) / (2.0 * L4)
    cos_beta = dot_p_u / r
    cos_beta = max(-1.0, min(1.0, cos_beta))
    alpha = math.atan2(M3[1], M3[0])
    beta = math.acos(cos_beta)

    candidate_dirs = [
        (math.cos(alpha + beta), math.sin(alpha + beta)),
        (math.cos(alpha - beta), math.sin(alpha - beta)),
    ]

    best = None
    best_score = None

    for ux, uy in candidate_dirs:
        u = (ux, uy)
        J2 = (M3[0] + L4 * u[0], M3[1] + L4 * u[1])
        J3 = (M3[0] + L4_J3 * u[0], M3[1] + L4_J3 * u[1])

        dx = J3[0] - M2[0]
        dy = J3[1] - M2[1]
        d = math.hypot(dx, dy)
        if d < 1e-9:
            continue

        # Intersection of circles centered at M2 and J3, with crank and coupler radii.
        a = (d**2 + L2**2 - L3**2) / (2.0 * d)
        h = math.sqrt(max(L2**2 - a**2, 0.0))

        vx = dx / d
        vy = dy / d
        px = -vy
        py = vx

        candidates_J1 = [
            (M2[0] + a * vx + h * px, M2[1] + a * vy + h * py),
            (M2[0] + a * vx - h * px, M2[1] + a * vy - h * py),
        ]

        for J1 in candidates_J1:
            # Correct crossed-link branch invariant from Geometry.md.
            if cross2d(M1, J2, J1) <= 0 or cross2d(M2, J1, J2) >= 0:
                continue

            theta1 = math.atan2(J2[1] - M1[1], J2[0] - M1[0])
            theta2 = math.atan2(J1[1] - M2[1], J1[0] - M2[0])

            theta1_deg = math.degrees(theta1)
            theta2_deg = math.degrees(theta2)
            phi1_deg = theta1_deg
            phi2_deg = theta2_deg

            # Prefer the branch closest to the raw origin-based angles so the solver
            # remains stable without a full-down offset convention.
            score = abs(theta1_deg) + abs(theta2_deg)

            # J5 is positioned along the purple bar with a perpendicular offset.
            # This is only a visual reference for the rigid bar; it is not used in the IK itself.
            bar_dir = (J2[0] - M3[0], J2[1] - M3[1])
            bar_mag = math.hypot(*bar_dir)
            if bar_mag < 1e-9:
                raise ValueError("Degenerate bar direction in IK solution.")
            ux = bar_dir[0] / bar_mag
            uy = bar_dir[1] / bar_mag
            nx = -uy
            ny = ux
            J5 = (
                M3[0] + L4_J5 * ux + float(bar_length_j5_perp_offset) * nx,
                M3[1] + L4_J5 * uy + float(bar_length_j5_perp_offset) * ny,
            )

            plate_vec = (M3[0] - J2[0], M3[1] - J2[1])
            plate_angle_abs = math.atan2(plate_vec[1], plate_vec[0])
            plate_angle_deg = math.degrees(plate_angle_abs)

            candidate = {
                "M3": M3,
                "J1": J1,
                "J2": J2,
                "J3": J3,
                "J5": J5,
                "theta1_deg": theta1_deg,
                "theta2_deg": theta2_deg,
                "phi1_deg": phi1_deg,
                "phi2_deg": phi2_deg,
                "plate_angle_deg": plate_angle_deg,
                "m3_angle_deg": plate_angle_deg,
                "score": score,
            }

            if best is None or score < best_score:
                best = candidate
                best_score = score

    if best is None:
        raise ValueError(f"No valid IK branch found for M3 = ({x_rel}, {y_rel}).")

    best.pop("score", None)
    return best


def is_valid_crossed_branch_from_ik(solution):
    J1 = solution["J1"]
    J2 = solution["J2"]
    return cross2d(M1, J2, J1) > 0 and cross2d(M2, J1, J2) < 0


def draw_pose(ax, name: str, pose: dict) -> None:
    x_rel, y_rel = pose["M3"]
    solution = solve_ik_for_m3(x_rel, y_rel)
    M3 = solution["M3"]
    J1 = solution["J1"]
    J2 = solution["J2"]
    J3 = solution["J3"]
    J5 = solution["J5"]

    ax.plot([M1[0], J2[0]], [M1[1], J2[1]], "r-", linewidth=3)
    ax.plot([M2[0], J1[0]], [M2[1], J1[1]], "y-", linewidth=3)
    ax.plot([J1[0], J3[0]], [J1[1], J3[1]], "g-", linewidth=3)
    ax.plot([J2[0], J3[0]], [J2[1], J3[1]], "m-", linewidth=3)
    ax.plot([J2[0], M3[0]], [J2[1], M3[1]], "m-", linewidth=3)
    ax.plot([J4[0], J5[0]], [J4[1], J5[1]], "k--", linewidth=1.5, alpha=0.7)

    for label, point in [("M1", M1), ("M2", M2), ("J1", J1), ("J2", J2), ("J3", J3), ("M3", M3), ("J4", J4), ("J5", J5)]:
        ax.scatter(point[0], point[1], s=28, color="black")
        ax.text(point[0] + 2, point[1] + 2, label, fontsize=9)

    ax.set_title(f"STEM 5-bar IK: {name}\n(theta1={solution['theta1_deg']:.2f}°, theta2={solution['theta2_deg']:.2f}°)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")

    valid = is_valid_crossed_branch_from_ik(solution)
    ax.text(0.02, 0.97, "CROSSED BRANCH: OK" if valid else "CROSSED BRANCH: FAIL",
            transform=ax.transAxes, bbox=dict(facecolor="white", alpha=0.8), va="top")


def print_pose_summary() -> None:
    for name in ["home", "full_down", "full_up", "max_forward", "max_rearward"]:
        x_rel, y_rel = POSES[name]["M3"]
        sol = solve_ik_for_m3(x_rel, y_rel)
        ok = is_valid_crossed_branch_from_ik(sol)
        print(
            f"{name:>12}: M3=({sol['M3'][0]:.2f}, {sol['M3'][1]:.2f}), "
            f"theta1={sol['theta1_deg']:.2f}°, theta2={sol['theta2_deg']:.2f}°, "
            f"plate_angle={sol['plate_angle_deg']:.2f}°, valid_crossed={ok}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Solve the STEM 5-bar IK and visualize the resulting linkage geometry.")
    parser.add_argument("--x", type=float, default=None, help="Output point x coordinate (mm)")
    parser.add_argument("--y", type=float, default=None, help="Output point y coordinate (mm)")
    parser.add_argument("--pose", choices=list(POSES.keys()), help="Use one of the documented design poses")
    parser.add_argument("--all", action="store_true", help="Plot the five document edge poses using the IK solver")
    parser.add_argument("--save", type=str, default=None, help="Optional save path for the generated figure (PNG)")
    args = parser.parse_args()

    if args.x is not None or args.y is not None:
        if args.x is None or args.y is None:
            parser.error("Both --x and --y must be provided together.")
        sol = solve_ik_for_m3(args.x, args.y)
        print(f"input (x_rel, y_rel) = ({args.x:.2f}, {args.y:.2f})")
        print(f"M3 = ({sol['M3'][0]:.2f}, {sol['M3'][1]:.2f})")
        print(f"J1 = ({sol['J1'][0]:.2f}, {sol['J1'][1]:.2f})")
        print(f"J2 = ({sol['J2'][0]:.2f}, {sol['J2'][1]:.2f})")
        print(f"theta1_raw = {sol['theta1_deg']:.2f} deg")
        print(f"theta2_raw = {sol['theta2_deg']:.2f} deg")
        print(f"M1 output (full-down-zero) = {sol['phi1_deg']:.2f} deg")
        print(f"M2 output (full-down-zero) = {sol['phi2_deg']:.2f} deg")
        print(f"plate angle relative to full-down horizontal = {sol['plate_angle_deg']:.2f} deg")
        print(f"M3 angle relative to full-down horizontal = {sol['m3_angle_deg']:.2f} deg")

        fig, ax = plt.subplots(figsize=(8, 6))
        # Draw from the IK solution instead of hardcoded points.
        ax.plot([M1[0], sol['J2'][0]], [M1[1], sol['J2'][1]], "r-", linewidth=3)
        ax.plot([M2[0], sol['J1'][0]], [M2[1], sol['J1'][1]], "y-", linewidth=3)
        ax.plot([sol['J1'][0], sol['J3'][0]], [sol['J1'][1], sol['J3'][1]], "g-", linewidth=3)
        ax.plot([sol['J2'][0], sol['J3'][0]], [sol['J2'][1], sol['J3'][1]], "m-", linewidth=3)
        ax.plot([sol['J2'][0], sol['M3'][0]], [sol['J2'][1], sol['M3'][1]], "m-", linewidth=3)
        ax.plot([J4[0], sol['J5'][0]], [J4[1], sol['J5'][1]], "k--", linewidth=1.5, alpha=0.7)
        for label, point in [("M1", M1), ("M2", M2), ("J1", sol['J1']), ("J2", sol['J2']), ("J3", sol['J3']), ("M3", sol['M3']), ("J4", J4), ("J5", sol['J5'])]:
            ax.scatter(point[0], point[1], s=28, color="black")
            ax.text(point[0] + 2.0, point[1] + 2.0, label, fontsize=9)

        ax.set_title(f"IK result for input (x_rel, y_rel)=({args.x:.2f}, {args.y:.2f})")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_xlim(-120, 360)
        ax.set_ylim(-60, 760)
        if args.save:
            fig.savefig(args.save, dpi=200)
            print(f"Saved figure to: {Path(args.save).resolve()}")
        else:
            plt.show()
        return

    if args.pose:
        print_pose_summary()
        fig, ax = plt.subplots(figsize=(8, 6))
        if args.pose in POSES:
            pose = POSES[args.pose]
            draw_pose(ax, args.pose, {"M3": pose["M3"]})
        plt.tight_layout()
        if args.save:
            fig.savefig(args.save, dpi=200)
            print(f"Saved figure to: {Path(args.save).resolve()}")
        else:
            plt.show()
        return

    if args.all:
        print_pose_summary()
        fig, axes = plt.subplots(1, 5, figsize=(18, 4), sharex=True, sharey=True)
        for ax, pose_name in zip(axes, ["home", "full_down", "full_up", "max_forward", "max_rearward"]):
            draw_pose(ax, pose_name, POSES[pose_name])
        for ax in axes:
            ax.set_xlim(-120, 360)
            ax.set_ylim(-60, 760)
        fig.suptitle("STEM 5-bar IK edge pose check")
        plt.tight_layout()
        if args.save:
            fig.savefig(args.save, dpi=200)
            print(f"Saved figure to: {Path(args.save).resolve()}")
        else:
            plt.show()
        return

    # Default: solve and plot the full-down pose.
    print_pose_summary()
    fig, ax = plt.subplots(figsize=(8, 6))
    draw_pose(ax, "full_down", POSES["full_down"])
    plt.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=200)
        print(f"Saved figure to: {Path(args.save).resolve()}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
