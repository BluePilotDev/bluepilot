#!/usr/bin/env python3
import os
import glob
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from typing import Dict, List, Tuple, Any

import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, BatchNormalization, Dropout
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
from tensorflow.keras.optimizers import Adam
from sklearn.model_selection import train_test_split


class FordNNLCTrainer:
  """Train a Neural Network Lateral Controller for Ford vehicles"""

  def __init__(self, data_dir: str, model_output_dir: str, config: Dict[str, Any]):
    """
    Initialize the trainer

    Args:
        data_dir: Directory with extracted data in JSONL format
        model_output_dir: Directory to save trained model
        config: Configuration dictionary with hyperparameters
    """
    self.data_dir = data_dir
    self.model_output_dir = model_output_dir
    self.config = config

    # Create output directory
    os.makedirs(model_output_dir, exist_ok=True)

    # Initialize data structures
    self.features = None
    self.targets = None
    self.fingerprint_map = {}
    self.normalization_params = {}

    # Set random seeds for reproducibility
    np.random.seed(self.config.get("random_seed", 42))
    tf.random.set_seed(self.config.get("random_seed", 42))

  def load_data(self) -> None:
    """Load and combine extracted data from JSONL files"""
    # Find all data files
    data_files = glob.glob(os.path.join(self.data_dir, "*.jsonl"))
    if not data_files:
      raise ValueError(f"No data files found in {self.data_dir}")

    print(f"Found {len(data_files)} data files")

    # Load and combine data
    all_data = []
    for file_path in tqdm(data_files, desc="Loading data files"):
      with open(file_path, 'r') as f:
        for line in f:
          try:
            data_point = json.loads(line)
            all_data.append(data_point)
          except json.JSONDecodeError:
            print(f"Error parsing line in {file_path}")
            continue

    print(f"Loaded {len(all_data)} total data points")

    # Filter data based on criteria
    filtered_data = []
    for data in all_data:
      # Skip samples with human turns if configured to do so
      if self.config.get("exclude_human_turns", True) and data.get("human_turn", False):
        continue

      # Skip samples during lane changes if configured to do so
      if self.config.get("exclude_lane_changes", True) and data.get("lane_change_state", 0) > 0:
        continue

      # Only include active lateral control samples
      if self.config.get("only_active_control", True) and not bool(data.get("mode", 0)):
        continue

      # Add the sample
      filtered_data.append(data)

    print(f"Filtered to {len(filtered_data)} data points")

    # Process the data
    self._process_data(filtered_data)

  def _process_data(self, data: List[Dict[str, Any]]) -> None:
    """
    Process and normalize data for training

    Args:
        data: List of data points
    """
    # Extract features and targets
    feature_data = []
    target_data = []
    fingerprints = []

    continuous_features = ['v_ego_raw', 'steering_angle_deg', 'yaw_rate', 'current_curvature', 'desired_curvature', 'model_path_offset']

    # Additional features if available
    lane_line_features = [f'lane_line_{i}_prob' for i in range(4)]
    lane_line_std_features = [f'lane_line_{i}_std' for i in range(4)]

    # Boolean features to include
    boolean_features = ['left_blinker', 'right_blinker', 'human_turn']

    # Target outputs
    target_outputs = ['apply_curvature', 'desired_curvature_rate', 'path_offset', 'path_angle']

    for item in data:
      # Continuous features
      feature_vector = [float(item.get(feature, 0.0)) for feature in continuous_features]

      # Add lane line features if all are available
      if all(feature in item for feature in lane_line_features):
        for feature in lane_line_features + lane_line_std_features:
          feature_vector.append(float(item.get(feature, 0.0)))

      # Add boolean features
      for feature in boolean_features:
        feature_vector.append(1.0 if item.get(feature, False) else 0.0)

      # Target vector
      target_vector = [float(item.get(target, 0.0)) for target in target_outputs]

      # Fingerprint for one-hot encoding
      fingerprints.append(str(item.get('fingerprint', 'unknown')))

      feature_data.append(feature_vector)
      target_data.append(target_vector)

    # Validate dimensions
    if not all(len(x) == len(feature_data[0]) for x in feature_data):
      raise ValueError("Feature vectors have inconsistent dimensions")

    if not all(len(y) == len(target_data[0]) for y in target_data):
      raise ValueError("Target vectors have inconsistent dimensions")

    # Convert to numpy arrays
    feature_array = np.array(feature_data, dtype=np.float32)
    target_array = np.array(target_data, dtype=np.float32)

    # Create one-hot encoding for fingerprints
    unique_fingerprints = sorted(list(set(fingerprints)))
    self.fingerprint_map = {fp: idx for idx, fp in enumerate(unique_fingerprints)}
    fingerprint_indices = [self.fingerprint_map.get(fp, 0) for fp in fingerprints]
    fingerprint_onehot = tf.keras.utils.to_categorical(fingerprint_indices, num_classes=len(unique_fingerprints))

    # Combine features with fingerprint one-hot
    self.features = np.concatenate([feature_array, fingerprint_onehot], axis=1)
    self.targets = target_array

    # Normalize data
    self._normalize_data()

  def _normalize_data(self) -> None:
    """Normalize features and targets"""
    # Calculate normalization parameters
    feature_means = np.mean(self.features, axis=0)
    feature_stds = np.std(self.features, axis=0) + 1e-8  # Avoid division by zero

    target_means = np.mean(self.targets, axis=0)
    target_stds = np.std(self.targets, axis=0) + 1e-8

    # Apply normalization
    self.features = (self.features - feature_means) / feature_stds
    self.targets = (self.targets - target_means) / target_stds

    # Store normalization parameters for inference
    self.normalization_params = {
      "feature_means": feature_means.tolist(),
      "feature_stds": feature_stds.tolist(),
      "target_means": target_means.tolist(),
      "target_stds": target_stds.tolist(),
      "fingerprint_map": self.fingerprint_map,
      "num_fingerprints": len(self.fingerprint_map),
    }

  def build_model(self) -> tf.keras.Model:
    """
    Build neural network model architecture

    Returns:
        Compiled Keras model
    """
    # Get configuration
    input_dim = self.features.shape[1]
    output_dim = self.targets.shape[1]

    # Layer sizes
    hidden_layers = self.config.get("hidden_layers", [13, 8, 5])
    dropout_rate = self.config.get("dropout_rate", 0.1)

    # Create model
    model = Sequential()

    # Input layer
    model.add(Dense(hidden_layers[0], activation='relu', input_shape=(input_dim,)))
    model.add(BatchNormalization())

    # Hidden layers
    for units in hidden_layers[1:]:
      model.add(Dense(units, activation='relu'))
      model.add(BatchNormalization())
      if dropout_rate > 0:
        model.add(Dropout(dropout_rate))

    # Output layer
    model.add(Dense(output_dim, activation='linear'))

    # Compile model
    learning_rate = self.config.get("learning_rate", 0.001)
    model.compile(optimizer=Adam(learning_rate=learning_rate), loss='mse', metrics=['mae'])

    print(model.summary())
    return model

  def train_model(self) -> Tuple[tf.keras.Model, Dict[str, List[float]]]:
    """
    Train the neural network model

    Returns:
        Trained model and training history
    """
    # Split data
    x_train, x_val, y_train, y_val = train_test_split(
      self.features, self.targets, test_size=self.config.get("validation_split", 0.2), random_state=self.config.get("random_seed", 42)
    )

    # Build model
    model = self.build_model()

    # Callbacks
    callbacks = [
      EarlyStopping(monitor='val_loss', patience=self.config.get("early_stopping_patience", 5), restore_best_weights=True),
      ModelCheckpoint(os.path.join(self.model_output_dir, 'model_checkpoint.h5'), save_best_only=True, monitor='val_loss'),
      ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=3, min_lr=1e-6),
    ]

    # Train model
    history = model.fit(
      x_train,
      y_train,
      epochs=self.config.get("epochs", 50),
      batch_size=self.config.get("batch_size", 64),
      validation_data=(x_val, y_val),
      callbacks=callbacks,
      verbose=1,
    )

    # Evaluate on validation set
    val_loss, val_mae = model.evaluate(x_val, y_val)
    print(f"Validation Loss: {val_loss}, Validation MAE: {val_mae}")

    return model, history.history

  def save_model(self, model: tf.keras.Model) -> None:
    """
    Save the trained model in formats suitable for on-device inference

    Args:
        model: Trained Keras model
    """
    # Save TensorFlow model
    model.save(os.path.join(self.model_output_dir, 'nnlc_model'))

    # Save weights and architecture as JSON
    weights = []
    biases = []

    for layer in model.layers:
      if isinstance(layer, Dense):
        layer_weights = layer.get_weights()
        weights.append(layer_weights[0].tolist())
        biases.append(layer_weights[1].tolist())

    # Create model JSON
    model_json = {
      "weights": weights,
      "biases": biases,
      "normalization": self.normalization_params,
      "architecture": [layer.output_shape[-1] for layer in model.layers if isinstance(layer, Dense)],
    }

    # Save model JSON
    json_path = os.path.join(self.model_output_dir, 'nnlc_model.json')
    with open(json_path, 'w') as f:
      json.dump(model_json, f)

    print(f"Model saved to {self.model_output_dir}")

  def plot_training_history(self, history: Dict[str, List[float]]) -> None:
    """
    Plot training history

    Args:
        history: Training history dictionary
    """
    plt.figure(figsize=(12, 5))

    # Plot loss
    plt.subplot(1, 2, 1)
    plt.plot(history['loss'], label='Training Loss')
    plt.plot(history['val_loss'], label='Validation Loss')
    plt.title('Model Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()

    # Plot MAE
    plt.subplot(1, 2, 2)
    plt.plot(history['mae'], label='Training MAE')
    plt.plot(history['val_mae'], label='Validation MAE')
    plt.title('Model MAE')
    plt.xlabel('Epoch')
    plt.ylabel('MAE')
    plt.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(self.model_output_dir, 'training_history.png'))
    plt.close()

  def evaluate_model(self, model: tf.keras.Model) -> None:
    """
    Evaluate the model performance in more detail

    Args:
        model: Trained model
    """
    # Split data
    _, x_test, _, y_test = train_test_split(
      self.features,
      self.targets,
      test_size=0.1,
      random_state=self.config.get("random_seed", 42) + 1,  # Different seed from training split
    )

    # Make predictions
    y_pred = model.predict(x_test)

    # Un-normalize predictions and targets
    target_means = np.array(self.normalization_params["target_means"])
    target_stds = np.array(self.normalization_params["target_stds"])

    y_test_denorm = y_test * target_stds + target_means
    y_pred_denorm = y_pred * target_stds + target_means

    # Calculate metrics for each output
    output_names = ['apply_curvature', 'desired_curvature_rate', 'path_offset', 'path_angle']
    metrics = {}

    for i, name in enumerate(output_names):
      mse = np.mean((y_test_denorm[:, i] - y_pred_denorm[:, i]) ** 2)
      mae = np.mean(np.abs(y_test_denorm[:, i] - y_pred_denorm[:, i]))

      metrics[name] = {'mse': float(mse), 'mae': float(mae)}

      print(f"{name}: MSE = {mse:.6f}, MAE = {mae:.6f}")

    # Save metrics
    with open(os.path.join(self.model_output_dir, 'evaluation_metrics.json'), 'w') as f:
      json.dump(metrics, f, indent=2)

    # Plot predictions vs. actual
    plt.figure(figsize=(15, 10))

    for i, name in enumerate(output_names):
      plt.subplot(2, 2, i + 1)
      plt.scatter(y_test_denorm[:, i], y_pred_denorm[:, i], alpha=0.5)
      plt.plot([-1, 1], [-1, 1], 'r--')  # Diagonal line for perfect predictions
      plt.title(f'{name} - Actual vs. Predicted')
      plt.xlabel('Actual')
      plt.ylabel('Predicted')

    plt.tight_layout()
    plt.savefig(os.path.join(self.model_output_dir, 'prediction_scatter.png'))
    plt.close()

  def run_pipeline(self) -> None:
    """Run the full training pipeline"""
    print(f"Starting training pipeline with config: {self.config}")

    # Load and process data
    self.load_data()

    # Train model
    model, history = self.train_model()

    # Save model
    self.save_model(model)

    # Plot training history
    self.plot_training_history(history)

    # Evaluate model
    self.evaluate_model(model)

    print("Training pipeline completed successfully")


def main():
  parser = argparse.ArgumentParser(description='Train Ford NNLC model')
  parser.add_argument('--data_dir', type=str, required=True, help='Directory containing extracted data files')
  parser.add_argument('--output_dir', type=str, default='./bluepilot_models', help='Directory to save model outputs')
  parser.add_argument('--config', type=str, default=None, help='JSON config file with hyperparameters')

  args = parser.parse_args()

  # Load config
  config = {
    "random_seed": 42,
    "epochs": 50,
    "batch_size": 64,
    "learning_rate": 0.001,
    "hidden_layers": [13, 8, 5],
    "dropout_rate": 0.1,
    "validation_split": 0.2,
    "early_stopping_patience": 5,
    "exclude_human_turns": True,
    "exclude_lane_changes": True,
    "only_active_control": True,
  }

  if args.config:
    with open(args.config, 'r') as f:
      config.update(json.load(f))

  # Create and run trainer
  trainer = FordNNLCTrainer(args.data_dir, args.output_dir, config)
  trainer.run_pipeline()


if __name__ == "__main__":
  main()
