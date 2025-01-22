import pandas as pd
import matplotlib.pyplot as plt
import re

# Load the CSV file
df = pd.read_csv("benchmark_results.csv")

# Function to extract tree type and iteration count
def parse_benchmark_name(name):
    match = re.match(r"(BM_[^<]+)<([^>]+)>/(\d+)", name)
    if match:
        benchmark, tree_type, iterations = match.groups()
        return benchmark, tree_type, int(iterations)
    return None, None, None

# Apply the function to extract components
df[["benchmark", "tree_type", "iterations"]] = df["name"].apply(
    lambda x: pd.Series(parse_benchmark_name(x))
)

# Define colors for benchmarks
benchmark_colors = {
    "BM_ComputeDescendants": "tab:blue",
    "BM_RemoveSubtree": "tab:orange"
}

# Create a single plot for both benchmarks
plt.figure(figsize=(10, 6))

for benchmark in ["BM_ComputeDescendants", "BM_RemoveSubtree"]:
    for tree_type in ["EdgeTreeVec", "EdgeTreeMap"]:
        subset = df[(df["benchmark"] == benchmark) & (df["tree_type"] == tree_type)]
        if not subset.empty:
            plt.plot(
                subset["iterations"],
                subset["real_time"],
                marker="o",
                linestyle="--" if tree_type == "EdgeTreeMap" else "-",
                color=benchmark_colors[benchmark],
                label=f"{benchmark} ({tree_type})"
            )

plt.xlabel("Iterations (N)")
plt.ylabel("Time (ns)")
plt.xscale("log")  # Logarithmic scale for better readability
plt.yscale("log")  # Log scale if times vary significantly
plt.legend()
plt.title("Benchmark Comparison: ComputeDescendants & RemoveSubtree")
plt.grid(True, which="both", linestyle="--", linewidth=0.5)

plt.savefig("benchmark_plot.pdf", format="pdf", bbox_inches="tight")
