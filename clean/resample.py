import pandas as pd
import numpy as np

sample_rate = 1000

def resample(name):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(name + '_clean.csv')

    # Generate a new series of counters
    new_counters = np.arange(0, df['counter'].max()+1, sample_rate)

    # Create a new dataframe with the new counters
    new_df = pd.DataFrame({'counter': new_counters})

    # Append the new dataframe to the existing dataframe
    df = df.append(new_df)

    # Sort the dataframe by counter
    df = df.sort_values('counter')

    # Fill NA values with last
    df.fillna(method='ffill', inplace=True)
    
    df = df[df['counter'] % sample_rate == 0]

    # Print the DataFrame
    df.to_csv(name + '_resampled.csv', index=False)
    
name = "random_walk"
resample(name)

# for x in range(10):
#     name = "data_log_" + str(x)
#     resample(name)