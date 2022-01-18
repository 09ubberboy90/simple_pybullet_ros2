## Data

Data is stored in as multiple folder called {simulation_name}

The structure of each folder is:

- {simulation_name}/cpu/ : This folder contains multiple .csv file numbered from 0 to X containing the cpu usage of each X run.
- {simulation_name}/log/ : This folder contains multiple .txt file numbered from 0 to X containing the log of each process for each X run.
- {simulation_name}/ram/ : This folder contains multiple .csv file numbered from 0 to X containing the memory usage of each X run.
- {simulation_name}/run.txt : This file contains the results of each simulation.
- {simulation_name}/{simulation_name}_no_smooth.csv : This file contains the raw non smoothed out graph created by the grapher file.
- {simulation_name}/{simulation_name}_smooth.csv : This file contains the smoothed out graph created by the grapher file.