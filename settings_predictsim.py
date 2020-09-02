def getSettings_predictsim_no_mtp():
    settings = {
            '0': {'contactConfiguration': '3',
                  'guessType': 'dataDriven',
                  'targetSpeed': 1.33,
                  'N': 50,
                  'tol': 4},
            '1': {'contactConfiguration': '0',
                  'guessType': 'dataDriven',
                  'targetSpeed': 1.33,
                  'N': 50,
                  'tol': 4},
            '2': {'contactConfiguration': '3',
                  'guessType': 'quasiRandom',
                  'targetSpeed': 1.33,
                  'N': 50,
                  'tol': 4},
            '3': {'contactConfiguration': '0',
                  'guessType': 'quasiRandom',
                  'targetSpeed': 1.33,
                  'N': 50,
                  'tol': 4}}
    return settings

def getSettings_predictsim_mtp():
    settings = {
            '0': {'guessType': 'dataDriven',
                  'targetSpeed': 1.33,
                  'N': 50,
                  'tol': 4},            
            '1': {'guessType': 'quasiRandom',
                  'targetSpeed': 1.33,
                  'N': 50,
                  'tol': 4}}
    return settings
