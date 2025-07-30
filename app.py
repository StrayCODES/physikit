import streamlit as st
import numpy as np
import matplotlib.pyplot as plt 

st.title('Physikit- creative physics + Data science toolkit')

#Example: Plotting a sine wave
x = np.linspace(0, 2 * np.pi, 100)
y= np.sin(x)

fig, ax = plt.subplots()
ax.plot(x, y)
ax.set_title('Y = sin(X)')

st.pyplot(fig)