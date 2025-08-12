
# PhysiKit

PhysiKit is an interactive, multi-page physics simulation toolkit built with Streamlit. It features a landing page and a dedicated simulation page, allowing users to explore classic and chaotic systems with scrollable, zoomable graphs and downloadable data.

## Features
- **Multi-page Streamlit App**: Landing page (`Home.py`) and simulation page (`pages/Simulations.py`).
- **Spring-Mass System**: Simulate oscillations, damping, and energy.
- **Double Pendulum**: Explore chaos and sensitivity to initial conditions.
- **Interactive Plots**: Infinite, scrollable, zoomable graphs powered by Plotly.
- **Manual Parameter Entry**: No limits on simulation parameters; enter any value.
- **Downloadable Data**: Export results as CSV for further analysis.
- **Extensible Structure**: Easily add new systems and features.

## Installation
1. Clone the repository:
   ```powershell
   git clone https://github.com/StrayCODES/physikit.git
   cd physikit
   ```
2. Install dependencies:
   ```powershell
   pip install -r requirements.txt
   ```

## Usage
Run the multi-page Streamlit app:
```powershell
streamlit run Home.py
```
Use the sidebar to navigate between the landing page and simulations. Adjust parameters, run simulations, and download results interactively.

## Project Structure
```
physikit/
├── Home.py                # Landing page (main entry)
├── pages/
│   └── Simulations.py     # Simulation page (multi-system)
├── requirements.txt       # Python dependencies
├── ml/                    # (Reserved) Machine learning utilities
├── systems/               # Physical system models
│   ├── spring_mass.py     # Spring-mass system simulation
│   └── double_pendulum.py # Double pendulum simulation
├── utils/                 # Utility functions (empty for now)
└── Readme.md              # Project documentation
```

## Extending PhysiKit
- Add new physical systems in the `systems/` directory and register them in `Simulations.py`.
- Implement ML features in the `ml/` directory.
- Contribute via pull requests and issues.

## License
MIT License

---
Created by StrayCODES
