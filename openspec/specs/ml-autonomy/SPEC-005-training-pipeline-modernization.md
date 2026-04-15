# SPEC-005: Training Data Pipeline Modernization

**Priority:** P0
**Category:** ML/Autonomy
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

The imitation learning pipeline is critically outdated:

- Uses **TensorFlow 1.6 and Keras 2.1.2** (released 2018, deprecated)
- `train_model.py` calls deprecated `fit_generator()` (line 93)
- `Generator.py` uses deprecated `keras.layers.advanced_activations.ELU`
- `np.fromstring()` used in `utils.py:10-14` (deprecated, slower than `np.frombuffer()`)
- Data augmentation limited to brightness and horizontal flips
- No modern framework support (PyTorch, TensorFlow 2.x, JAX)
- No dataset versioning, no experiment tracking
- Hardcoded ROI `[78,144,27,227]` in `Generator.py:52-54`
- 95% zero-label dropping ratio hardcoded (`zero_drop_percentage=0.95`)

## Proposed Solution

### 1. Framework-Agnostic Data Pipeline

```python
class CosysAirSimDataset:
    """Framework-agnostic dataset class with lazy loading."""

    def __init__(self, root_dir, transform=None):
        self.metadata = self._load_metadata(root_dir)
        ...

    def __getitem__(self, idx) -> dict:
        return {
            "image": self._load_image(idx),
            "depth": self._load_depth(idx),
            "lidar": self._load_lidar(idx),
            "pose": self._load_pose(idx),
            "controls": self._load_controls(idx),
            "metadata": self._load_frame_metadata(idx),
        }

    def to_torch(self) -> "torch.utils.data.Dataset":
        ...

    def to_tensorflow(self) -> "tf.data.Dataset":
        ...

    def to_huggingface(self) -> "datasets.Dataset":
        ...
```

### 2. Modern Data Augmentation

```python
class AugmentationPipeline:
    """Composable augmentation pipeline."""
    transforms = [
        RandomBrightness(range=(-0.3, 0.3)),
        RandomContrast(range=(0.7, 1.3)),
        RandomSaturation(range=(0.5, 1.5)),
        RandomGaussianNoise(std=(0, 0.05)),
        RandomMotionBlur(kernel_range=(3, 7)),
        RandomShadow(num_shadows=(1, 3)),
        RandomFog(intensity=(0, 0.3)),
        RandomCameraShift(x_range=(-10, 10), y_range=(-5, 5)),
        CoarseDropout(max_holes=8, max_size=20),
    ]
```

### 3. Experiment Tracking Integration

```python
# Built-in support for popular tracking tools
class ExperimentTracker:
    backends = ["wandb", "mlflow", "tensorboard"]

    def log_config(self, config: dict): ...
    def log_metrics(self, metrics: dict, step: int): ...
    def log_model(self, model, name: str): ...
    def log_dataset_info(self, dataset: CosysAirSimDataset): ...
```

### 4. Fix Deprecated APIs

- Replace `np.fromstring()` → `np.frombuffer()` in `utils.py`
- Remove TF1/Keras2 code, provide PyTorch examples
- Update `airsim_rec.txt` format to include sensor metadata

## Acceptance Criteria

- [ ] `CosysAirSimDataset` class supporting PyTorch, TensorFlow 2, and HuggingFace
- [ ] Data augmentation pipeline with at least 10 transforms
- [ ] Recording format includes sensor calibration and environment metadata
- [ ] `np.fromstring()` replaced everywhere with `np.frombuffer()`
- [ ] Example training scripts in PyTorch (imitation learning + RL fine-tuning)
- [ ] Experiment tracking with at least W&B and TensorBoard backends
- [ ] Dataset versioning with SHA256 manifests
- [ ] Balanced sampling utilities replacing hardcoded drop percentages

## Files Affected

- `PythonClient/imitation_learning/` — complete rewrite
- `PythonClient/cosysairsim/utils.py` — fix deprecated numpy calls
- New: `PythonClient/cosysairsim/data/`
- New: `PythonClient/cosysairsim/augmentation.py`
- New: `PythonClient/cosysairsim/tracking.py`
