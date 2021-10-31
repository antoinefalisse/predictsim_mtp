# 3D Predictive Simulations of Walking

This repository contains code and data to generate predictive simulations of human walking as described in "Modeling toes contributes to realistic stance knee mechanics in three-dimensional predictive simulations of walking". The paper has been submitted for publication. You can find the pre-print [here](https://www.biorxiv.org/content/10.1101/2021.08.13.456292v1).

The main script is: `main.py`. The easiest is to start exploring the code from there.

A lot of cleaning is still required to make this code user-friendly. I will work on that in the coming weeks. Please post an issue on GitHub if you would like to use this code already and need help.

# Install requirements

- Open Anaconda prompt
- Create environment: `conda create -n 3dpredsim pip spyder`
- Activate environment: `activate 3dpredsim`
- Navigate to the folder where you want to download the code: eg. `cd Documents`
- Download code: `git clone https://github.com/antoinefalisse/predictsim_mtp.git`
- Navigate to the folder: `cd predictsim_mtp`
- Checkout the cleaning branch: `git checkout cleaning`
- Install required packages: `python -m pip install -r requirements.txt`
- Run main code: `python main.py`
