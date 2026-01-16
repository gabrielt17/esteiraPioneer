import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
df = pd.read_csv("teleplot_2026-1-16_16-25.json")

step = 60 / (0.1 * 28)  # 21.43 RPM

bins = np.arange(
    df["RPM"].min() - step/2,
    df["RPM"].max() + step,
    step
)

plt.figure()
plt.hist(df["RPM"], bins=bins)
plt.xlabel("RPM")
plt.ylabel("Contagem")
plt.title("Histograma de RPM (bins alinhados à quantização)")
plt.show()

