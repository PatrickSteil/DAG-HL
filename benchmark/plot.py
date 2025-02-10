import pandas as pd
import matplotlib.pyplot as plt
import re

# Function to extract tree type and iteration count from the benchmark name
def parse_benchmark_name(name):
    match = re.match(r"(BM_[^<]+)<([^>]+)>/(\d+)", name)
    if match:
        benchmark, tree_type, iterations = match.groups()
        return benchmark, tree_type, int(iterations)
    return None, None, None

# Function to process the CSV and extract relevant columns
def load_and_process_data(file_path):
    df = pd.read_csv(file_path)
    df[["benchmark", "tree_type", "iterations"]] = df["name"].apply(
        lambda x: pd.Series(parse_benchmark_name(x))
    )
    return df

def plot_benchmark_results(df, benchmarks, tree_types, benchmark_colors):
    plt.figure(figsize=(10, 6))

    # Define markers for the two tree types
    markers = {
        "EdgeTreeVec": "o",  # O for Vec
        "EdgeTreeMap": "X"   # X for Map
    }

    for benchmark in benchmarks:
        for tree_type in tree_types:
            subset = df[(df["benchmark"] == benchmark) & (df["tree_type"] == tree_type)]
            if not subset.empty:
                plt.plot(
                    subset["iterations"],
                    subset["real_time"],
                    marker=markers[tree_type],
                    linestyle="--" if tree_type == "EdgeTreeMap" else "-",
                    color=benchmark_colors.get(benchmark, "tab:gray"),
                    label=f"{benchmark} ({tree_type})"
                )

    plt.xlabel("Iterations (N)")
    plt.ylabel("Time (ns)")
    plt.xscale("log")  # Logarithmic scale for better readability
    plt.yscale("log")  # Log scale if times vary significantly
    plt.legend()
    plt.title("Benchmark Comparison: ComputeDescendants & RemoveSubtree")
    plt.grid(True, which="both", linestyle="--", linewidth=0.5)

# Main execution
if __name__ == "__main__":
    # Load and process the data
    df = load_and_process_data("benchmark_results.csv")
    
    # Define benchmarks and tree types for plotting
    benchmarks = ["BM_ComputeDescendants_Seq", "BM_ComputeDescendants_Rand", "BM_RemoveSubtree_Seq", "BM_Remove100Subtree_Rand"]
    tree_types = ["EdgeTreeVec", "EdgeTreeMap"]
    
    # Define colors for benchmarks
    benchmark_colors = {
        "BM_ComputeDescendants_Seq": "tab:blue",
        "BM_ComputeDescendants_Rand": "tab:green",
        "BM_RemoveSubtree_Seq": "tab:orange",
        "BM_Remove100Subtree_Rand": "tab:red"
    }

    # Plot the benchmark results
    plot_benchmark_results(df, benchmarks, tree_types, benchmark_colors)
    
    # Save the plot to a PDF file
    plt.savefig("benchmark_plot.pdf", format="pdf", bbox_inches="tight")
