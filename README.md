# 3D Predictive Simulations of Walking

This repository contains code and data to generate predictive simulations of human walking as described in "_Modeling toes contributes to realistic stance knee mechanics in three-dimensional predictive simulations of walking_". The paper has been submitted for publication. You can find the pre-print [here](https://www.biorxiv.org/content/10.1101/2021.08.13.456292v1).

The main script is: `main.py`. The easiest is to start exploring the code from there.

# Install requirements

- Open Anaconda prompt
- Create environment: `conda create -n 3dpredsim pip spyder`
- Activate environment: `activate 3dpredsim`
- Navigate to the folder where you want to download the code: eg. `cd Documents`
- Download code: `git clone https://github.com/antoinefalisse/predictsim_mtp.git`
- Navigate to the folder: `cd predictsim_mtp`
- Install required packages: `python -m pip install -r requirements.txt`

# Run simulations and plot results:
- Run main: `python main.py`
- Run plotResults: `python plotResults.py`
