import pandas as pd

def clean(name):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(name + '.csv')

    # Get all the rows of the first occurrence of each unique value in the 'total_value' column
    df_unique = df.drop_duplicates(subset='total_value', keep='first')

    # Print the DataFrame
    df_unique.to_csv(name + '_clean.csv', index=False)

name = "data_log_0"
clean(name)

# for x in range(10):
#     name = "data_log_" + str(x)
#     clean(name)