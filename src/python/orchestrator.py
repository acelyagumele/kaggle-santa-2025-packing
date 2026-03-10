import os
import sys
import shutil
import random
import argparse
import subprocess
import pandas as pd
import numpy as np
from decimal import Decimal, getcontext
from shapely import affinity
from shapely.geometry import Polygon
from shapely.strtree import STRtree

# I enforce strict 25-digit precision to prevent Kaggle evaluation mismatches
getcontext().prec = 25
SCALE_FACTOR = Decimal("1e18")

class ChristmasTree:
    """Represents a single, rotatable polygon mapped with arbitrary precision."""
    def __init__(self, center_x="0", center_y="0", angle="0"):
        self.center_x = Decimal(center_x)
        self.center_y = Decimal(center_y)
        self.angle = Decimal(angle)

        trunk_w = Decimal("0.15"); trunk_h = Decimal("0.2")
        base_w = Decimal("0.7"); mid_w = Decimal("0.4"); top_w = Decimal("0.25")
        tip_y = Decimal("0.8"); tier_1_y = Decimal("0.5"); tier_2_y = Decimal("0.25")
        base_y = Decimal("0.0"); trunk_bottom_y = -trunk_h

        initial_polygon = Polygon([
            (Decimal("0.0") * SCALE_FACTOR, tip_y * SCALE_FACTOR),
            (top_w / Decimal("2") * SCALE_FACTOR, tier_1_y * SCALE_FACTOR),
            (top_w / Decimal("4") * SCALE_FACTOR, tier_1_y * SCALE_FACTOR),
            (mid_w / Decimal("2") * SCALE_FACTOR, tier_2_y * SCALE_FACTOR),
            (mid_w / Decimal("4") * SCALE_FACTOR, tier_2_y * SCALE_FACTOR),
            (base_w / Decimal("2") * SCALE_FACTOR, base_y * SCALE_FACTOR),
            (trunk_w / Decimal("2") * SCALE_FACTOR, base_y * SCALE_FACTOR),
            (trunk_w / Decimal("2") * SCALE_FACTOR, trunk_bottom_y * SCALE_FACTOR),
            (-(trunk_w / Decimal("2")) * SCALE_FACTOR, trunk_bottom_y * SCALE_FACTOR),
            (-(trunk_w / Decimal("2")) * SCALE_FACTOR, base_y * SCALE_FACTOR),
            (-(base_w / Decimal("2")) * SCALE_FACTOR, base_y * SCALE_FACTOR),
            (-(mid_w / Decimal("4")) * SCALE_FACTOR, tier_2_y * SCALE_FACTOR),
            (-(mid_w / Decimal("2")) * SCALE_FACTOR, tier_2_y * SCALE_FACTOR),
            (-(top_w / Decimal("4")) * SCALE_FACTOR, tier_1_y * SCALE_FACTOR),
            (-(top_w / Decimal("2")) * SCALE_FACTOR, tier_1_y * SCALE_FACTOR),
        ])
        rotated = affinity.rotate(initial_polygon, float(self.angle), origin=(0, 0))
        self.polygon = affinity.translate(rotated, xoff=float(self.center_x * SCALE_FACTOR), yoff=float(self.center_y * SCALE_FACTOR))

def load_group_from_df(n, df):
    """
    I designed this extractor to accurately match group IDs, 
    handling both zero-padded ('001_1') and raw ('1_1') formats.
    """
    group_data = df[df["id"].apply(lambda x: int(str(x).split('_')[0])) == int(n)]
    trees = []
    for _, row in group_data.iterrows():
        x = row["x"][1:] if str(row["x"]).startswith('s') else row["x"]
        y = row["y"][1:] if str(row["y"]).startswith('s') else row["y"]
        deg = row["deg"][1:] if str(row["deg"]).startswith('s') else row["deg"]
        trees.append(ChristmasTree(x, y, deg))
    return trees

def get_score(trees, n):
    """Calculates the Kaggle metric: (Max Bounding Box Side)^2 / Number of Trees"""
    if not trees: return 0.0
    xys = np.concatenate([np.asarray(t.polygon.exterior.xy).T / float(SCALE_FACTOR) for t in trees])
    min_x, min_y = xys.min(axis=0)
    max_x, max_y = xys.max(axis=0)
    score = max(max_x - min_x, max_y - min_y) ** 2
    return float(score / n)

def has_overlap(trees):
    """Strict validation engine using STRtree for O(N log N) spatial querying."""
    if len(trees) <= 1: return False
    polygons = [t.polygon for t in trees]
    tree_index = STRtree(polygons)
    for i, poly in enumerate(polygons):
        indices = tree_index.query(poly)
        for idx in indices:
            if idx == i: continue
            if poly.intersects(polygons[idx]) and not poly.touches(polygons[idx]):
                return True
    return False

def eval_df_sub(df, verb=False):
    """Evaluates the entire submission file to establish the baseline Global Score."""
    failed = []
    total_score = 0.0
    for n in range(1, 201):
        trees = load_group_from_df(n, df)
        if not trees: continue
        score = get_score(trees, n)
        total_score += score
        if verb: print(f"{n:3}  {score:.6f}")
        if has_overlap(trees): failed.append(n)
        
    if len(failed) == 0:
        print(" Initial overlap check was successful.")
    else:
        print(" WARNING: Overlap detected in the initial file! Groups:", *failed)
    return total_score

def main():
    # I use parse_known_args to gracefully ignore hidden environment variables (e.g., in Colab)
    parser = argparse.ArgumentParser()
    parser.add_argument("--solver", type=str, default="./mysolver")
    parser.add_argument("--csv", type=str, default="submission.csv")
    args, unknown = parser.parse_known_args()

    solver_path = args.solver
    target_csv = args.csv
    partial_csv = "partial_out.csv" 

    if not os.path.exists(target_csv):
        print(f"[FATAL ERROR] {target_csv} not found. Please upload it.")
        sys.exit(1)

    print("Initiating Orchestration Pipeline...")
    df = pd.read_csv(target_csv)
    
    # Pre-calculate and cache baseline scores
    global_score = eval_df_sub(df, verb=False)
    print(f"Initial Score: {global_score:.12f}")

    group_scores = {}
    for n in range(1, 201):
        trees = load_group_from_df(n, df)
        group_scores[n] = get_score(trees, n) if trees else 0.0

    iteration = 0
    while True:
        iteration += 1
        n = random.randint(1, 200)
        rng_seed = random.randint(10, 10000)
        
        print(f"[{iteration}] ⚙️  Dispatching C++ Worker -> Group: {n} | Seed: {rng_seed}")
        
        # 1. Execute C++ Engine
        try:
            subprocess.run(
                [solver_path, "-n", str(n), "-r", str(rng_seed), "-i", "50000", "-o", partial_csv],
                check=True, timeout=60
            )
        except Exception as e:
            print(f"[WARNING] C++ Engine failed or timed out. Detail: {e}")
            continue

        # 2. Arbiter Phase
        try:
            with open(partial_csv, 'r') as f:
                first_line = f.readline().strip()
                
            if first_line.startswith('id'):
                partial_df = pd.read_csv(partial_csv)
                is_full_file = len(partial_df) > 500
            else:
                partial_df = pd.read_csv(partial_csv, names=['id', 'x', 'y', 'deg'])
                is_full_file = False

            mutated_trees = load_group_from_df(n, partial_df) if is_full_file else []
            if not is_full_file:
                for _, row in partial_df.iterrows():
                    x = row["x"][1:] if str(row["x"]).startswith('s') else row["x"]
                    y = row["y"][1:] if str(row["y"]).startswith('s') else row["y"]
                    deg = row["deg"][1:] if str(row["deg"]).startswith('s') else row["deg"]
                    mutated_trees.append(ChristmasTree(x, y, deg))

        except Exception as e:
            print(f" [WARNING] Corrupted I/O on partial file. Detail: {e}")
            continue

        # 3. Validation 
        if has_overlap(mutated_trees):
            print(f"[REJECTED] Overlap detected by Shapely Arbiter. Group {n} reverted.")
        else:
            new_group_score = get_score(mutated_trees, n)
            old_group_score = group_scores[n]
            
            if new_group_score < old_group_score - 1e-9:
                improvement = old_group_score - new_group_score
                global_score -= improvement
                group_scores[n] = new_group_score
                print(f"[ACCEPTED] Group {n} improved by {improvement:.6f}. New Global: {global_score:.6f}")
                
                if is_full_file:
                    df = partial_df.copy()
                else:
                    for _, row in partial_df.iterrows():
                        df.loc[df['id'] == row['id'], ['x', 'y', 'deg']] = [row['x'], row['y'], row['deg']]
                
                df.to_csv(target_csv, index=False)

if __name__ == "__main__":
    main()