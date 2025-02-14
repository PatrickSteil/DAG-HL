import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("benchmark_bit_vector.csv")

df['Operation'] = df['name'].str.extract(r'BM_(VectorBool|BitVector)_(\w+)', expand=True)[1]
df['Type'] = df['name'].apply(lambda x: 'VectorBool' if 'VectorBool' in x else 'BitVector')
df['cpu_time_ns'] = df['cpu_time']

df['ShortName'] = df['name'].str.replace(r'BM_', '', regex=True)

operations = df['Operation'].dropna().unique()

fig, axes = plt.subplots(2, 2, figsize=(12, 8))
axes = axes.flatten()

for i, op in enumerate(operations):
    ax = axes[i]
    
    subset = df[df['Operation'] == op].copy()
    
    types = subset['Type'].unique()
    x_labels = subset['ShortName']
    x_pos = np.arange(len(x_labels))
    
    for t in types:
        data = subset[subset['Type'] == t]
        ax.bar(data['ShortName'], data['cpu_time_ns'], label=t, alpha=0.75)
    
    ax.set_title(op)
    ax.set_ylabel("Time (ns, log scale)")
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels, rotation=45, ha="right", fontsize=8)
    ax.set_yscale("log")
    ax.legend()

plt.tight_layout()
plt.savefig("benchmark_bit_vector.pdf")
plt.close()
