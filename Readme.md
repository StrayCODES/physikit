# PhysiKit

PhysiKit is an interactive physics simulation toolkit built with Streamlit. The current version focuses on simulating the classic spring-mass system, allowing users to explore dynamics, energy, and damping effects visually and numerically.

## Features
- **Spring-Mass System Simulation**: Adjust mass, spring constant, damping, initial position, and velocity.
- **Interactive Plots**: Visualize position, velocity, and energy over time.
- **Downloadable Data**: Export simulation results as CSV for further analysis.
- **Extensible Structure**: Modular codebase for adding more physical systems and machine learning features.

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
Run the Streamlit app:
```powershell
streamlit run app.py
```
Adjust simulation parameters in the sidebar and explore the results interactively.

## Project Structure
```
physikit/
├── app.py                # Main Streamlit application
├── requirements.txt      # Python dependencies
├── ml/                   # (Reserved) Machine learning utilities
├── systems/              # Physical system models
│   └── spring_mass.py    # Spring-mass system simulation
├── utils/                # Utility functions (empty for now)
└── Readme.md             # Project documentation
```

## Extending PhysiKit
- Add new physical systems in the `systems/` directory.
- Implement ML features in the `ml/` directory.
- Contribute via pull requests and issues.

## License
MIT License

---
Created by StrayCODES
