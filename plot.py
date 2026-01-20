import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob

# List of files provided by the user
file_names = [
    "51pwm.json", "102pwm.json", "153pwm.json", "204pwm.json", 
    "255pwm.json", "306pwm.json", "358pwm.json", "409pwm.json", 
    "460pwm.json", "511pwm.json"
]

all_errors = []

for file_name in file_names:
    try:
        # The file extension is .json but snippets suggest CSV format.
        # Trying to read as CSV.
        df = pd.read_csv(file_name)
        
        # Identify the error column
        # Snippets show 'error' or 'error(RPM)'
        error_col = None
        for col in df.columns:
            if "error" in col.lower():
                error_col = col
                break
        
        if error_col:
            all_errors.extend(df[error_col].dropna().tolist())
        else:
            print(f"Warning: No error column found in {file_name}")
            print(f"Columns: {df.columns}")

    except Exception as e:
        print(f"Error reading {file_name}: {e}")

# Convert to numpy array
errors = np.array(all_errors)

print(f"Total data points: {len(errors)}")
print(f"Mean error: {np.mean(errors):.2f}")
print(f"Std dev: {np.std(errors):.2f}")
print(f"Min: {np.min(errors)}, Max: {np.max(errors)}")

# Plotting
if len(errors) > 0:
    bin_width = 22 # Requested resolution
    
    # Determine range
    min_err = np.min(errors)
    max_err = np.max(errors)
    
    # Create bins centered at 0
    # We want bins like [-11, 11], [11, 33], [-33, -11] etc.
    # So edges are multiples of bin_width offset by -bin_width/2? No.
    # Center 0 -> interval [-11, 11]. Edges are at -11 and 11.
    # Next bin center 22 -> interval [11, 33].
    # So edges are k * 22 - 11.
    
    # Calculate number of bins needed to cover the range
    # Start from a multiple of width close to min
    start_bin = np.floor(min_err / bin_width) * bin_width - bin_width/2
    end_bin = np.ceil(max_err / bin_width) * bin_width + bin_width/2
    
    bins = np.arange(start_bin, end_bin + bin_width, bin_width)
    
    # Adjust to ensure 0 is exactly in the center of a bin
    # We want edges to be ... -33, -11, 11, 33 ...
    # This matches the formula: edge = k * 22 + 11 (or -11)
    # Let's verify start_bin
    
    # Re-calculate bins carefully to center on 0
    # Range of data
    limit = max(abs(min_err), abs(max_err)) + bin_width
    
    # Create edges: 0 +/- 11, then steps of 22
    # np.arange(start, stop, step)
    # We want ... -11, 11, ...
    
    # Let's generate edges from -Limit to +Limit
    # To center 0: edges should be ..., -33, -11, 11, 33, ...
    # Check if 0 is in [-11, 11]. Yes.
    
    # Find min edge
    n_min = np.floor((min_err - 11) / 22)
    min_edge = n_min * 22 + 11 
    # Wait, if min_err is -5, floor((-5-11)/22) = floor(-0.7) = -1. 
    # min_edge = -1*22 + 11 = -11. Range [-11, 11] covers -5. Correct.
    
    # Find max edge
    n_max = np.ceil((max_err - 11) / 22)
    max_edge = n_max * 22 + 11
    
    bins = np.arange(min_edge, max_edge + bin_width, bin_width)
    
    plt.figure(figsize=(10, 6))
    plt.hist(errors, bins=bins, color='skyblue', edgecolor='black', alpha=0.7)
    plt.axvline(0, color='red', linestyle='--', linewidth=2, label='Zero Error')
    plt.title('Histograma de Erro de RPM (Todos os Níveis de PWM)')
    plt.xlabel('Erro (RPM)')
    plt.ylabel('Frequência')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig('histogram_error_centered.png')
    plt.show()
else:
    print("No error data found.")