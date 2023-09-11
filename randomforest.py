import pandas as pd
import numpy as np
from sklearn import preprocessing
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
import seaborn as sns
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
from matplotlib import pyplot as plt
from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import RandomizedSearchCV
import pandas as pd

# Read the CSV files
df1 = pd.read_csv("flatgelsight_experiment_results_modified.csv")
df2 = pd.read_csv("geltip_experiment_results_modified.csv")
df3 = pd.read_csv("tactip_experiment_results_modified.csv")

# Combine the DataFrames
combined_df = pd.concat([df1, df2, df3], ignore_index=True)

# Write the combined DataFrame to a new CSV file
combined_df.to_csv("combined.csv", index=False)
combined_df.isnull().sum()
plt.figure(figsize=(8,6))
sns.countplot(x='Success_Count',data=combined_df)
plt.xlabel('Success_Count')
plt.ylabel('count')
plt.title('count plot for gripper prediction')
plt.show()
# Load the dataset
dataset = pd.read_csv('combined.csv')
dataset.replace({'Gripper':{'flatgelsight_gripper':1,'tactip_gripper':2,'geltip_gripper':3}},inplace=True)
label_encoder = preprocessing.LabelEncoder()


# Encode labels in column 'species'.
dataset['Object']= label_encoder.fit_transform(combined_df['Object'])

dataset['Object'].unique()

# Convert the string representation of lists to actual lists
dataset['Force Move XYZ'] = dataset['Force Move XYZ'].apply(lambda x: eval(x))

# Convert lists to individual columns
expanded_columns = pd.DataFrame(dataset['Force Move XYZ'].tolist(), columns=['Value_1', 'Value_2', 'Value_3'])
dataset = pd.concat([dataset, expanded_columns], axis=1)

# Drop the original 'Mixed_Column'
dataset.drop(columns=['Force Move XYZ'], inplace=True)
dataset.replace({'Gripper':{'flatgelsight_gripper':1,'tactip_gripper':2,'geltip_gripper':3}},inplace=True)
label_encoder = preprocessing.LabelEncoder()

# Encode labels in column 'species'.
dataset['Object']= label_encoder.fit_transform(combined_df['Object'])

dataset['Object'].unique()
X = dataset.drop('Gripper',axis=1)
y = dataset["Gripper"]
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
# Create a Random Forest classifier
rf_classifier = RandomForestClassifier(n_estimators=100, random_state=42)
# Train the classifier
rf_classifier.fit(X_train, y_train)
# Make predictions
y_pred = rf_classifier.predict(X_test)
# Evaluate the model
accuracy = accuracy_score(y_test, y_pred)
conf_matrix = confusion_matrix(y_test, y_pred)
classification_rep = classification_report(y_test, y_pred)

print(f"Accuracy: {accuracy:.2f}")
print("Confusion Matrix:")
print(conf_matrix)
print("Classification Report:")
print(classification_rep)
#graph of best gripper based on success_count
gs=combined_df.groupby(['Gripper','Success_Count']).size().unstack(fill_value=0)
gs.plot(kind='bar',stacked=True)
data = pd.read_excel('random_sample (1).xlsx')
# Convert the 'Force Move XYZ' column to lists
data['Force Move XYZ'] = data['Force Move XYZ'].apply(lambda x: eval(str(x)))
data['Force Move XYZ'] = data['Force Move XYZ'].apply(lambda x: eval(str(x)))
expanded_columns = pd.DataFrame(data['Force Move XYZ'].tolist(), columns=['Value_1', 'Value_2', 'Value_3'])
data = pd.concat([data, expanded_columns], axis=1)
data.drop(columns=['Force Move XYZ'], inplace=True)
# Encode the 'Gripper' and 'Object' columns
data.replace({'Gripper': {'flatgelsight_gripper': 1, 'tactip_gripper': 2, 'geltip_gripper': 3}}, inplace=True)
data['Object'] = label_encoder.fit_transform(data['Object'])
rf_classifier = RandomForestClassifier(n_estimators=100, random_state=42)
label_encoder = LabelEncoder()
all_labels = np.concatenate((y_train, y_test), axis=None)
label_encoder.fit(all_labels)
# Fit the RandomForestClassifier model
rf_classifier.fit(X_train, y_train)
# Make predictions using the trained model
X_new = data.drop('Gripper', axis=1)
# Make predictions using the trained model
predictions = rf_classifier.predict(X_new)
valid_label_range = np.arange(len(label_encoder.classes_))
valid_predictions = np.intersect1d(predictions, valid_label_range)
# Decode the numerical predictions back to original labels
predicted_grippers = label_encoder.inverse_transform(valid_predictions)
print("Predicted Grippers:")
print(predicted_grippers)
