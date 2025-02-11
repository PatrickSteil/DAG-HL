import pandas as pd
import matplotlib.pyplot as plt
import re

# Function to extract benchmark name and input size
def parse_benchmark_name(name):
    match = re.match(r"(BM_[^/]+)/(\d+)", name)
    if match:
        benchmark, size = match.groups()
        return benchmark, int(size)
    return None, None

# Function to process CSV and extract relevant columns
def load_and_process_data(file_path):
    df = pd.read_csv(file_path)
    df[["benchmark", "size"]] = df["name"].apply(
        lambda x: pd.Series(parse_benchmark_name(x))
    )
    return df.dropna(subset=["benchmark", "size"])  # Drop rows where parsing failed

def plot_benchmark_results(df, benchmarks, benchmark_colors):
    plt.figure(figsize=(10, 6))

    markers = {
        "BM_Intersect_Vector": "o",
        "BM_Intersect_CompressedVector": "X",
        "BM_Intersect_DeltaCompressedVector": "."
    }

    for benchmark in benchmarks:
        subset = df[df["benchmark"] == benchmark]
        if not subset.empty:
            plt.plot(
                subset["size"],
                subset["real_time"],
                marker=markers.get(benchmark, "s"),
                linestyle="--" if "Compressed" in benchmark else "-",
                color=benchmark_colors.get(benchmark, "tab:gray"),
                label=benchmark
            )

    plt.xlabel("Input Size (N)")
    plt.ylabel("Time (ns)")
    plt.xscale("log")
    plt.yscale("log")
    plt.legend()
    plt.title("Benchmark: Vector vs DeltaVector vs CompressedVector vs DeltaCompressedVector Intersection")
    plt.grid(True, which="both", linestyle="--", linewidth=0.5)

# Main execution
if __name__ == "__main__":
    df = load_and_process_data("benchmark_compress_vector.csv")

    benchmarks = ["BM_Intersect_Vector", "BM_Intersect_DeltaVector", "BM_Intersect_CompressedVector", "BM_Intersect_DeltaCompressedVector"]

    benchmark_colors = {
        "BM_Intersect_Vector": "tab:blue",
        "BM_Intersect_DeltaVector": "tab:orange",
        "BM_Intersect_CompressedVector": "tab:red",
        "BM_Intersect_DeltaCompressedVector": "tab:green"
    }

    plot_benchmark_results(df, benchmarks, benchmark_colors)

    plt.savefig("benchmark_intersect_plot.pdf", format="pdf", bbox_inches="tight")
    plt.show()

