# Datasets for Maritime Robotics

Open-access datasets are essential for developing and validating perception, navigation, and control algorithms. This page catalogs available datasets for maritime robotics research.

## Multi-Sensor Datasets

### REMARO Data

**[REMARO Data Collections](https://github.com/remaro-network/remaro_data)**

The REMARO (Reliable AI for Marine Robotics) network provides various datasets for marine robotics.

**Contents:**
- Multi-sensor recordings
- Underwater vehicle data
- Environmental conditions
- Ground truth when provided by the dataset

**Use Cases:**
- Algorithm benchmarking
- Sensor fusion research
- Environmental monitoring studies

**Format:** ROS bags, raw sensor data

## Sonar Datasets

### OpenSonarDatasets

**[OpenSonarDatasets](https://github.com/remaro-network/OpenSonarDatasets)**

Curated collection of sonar datasets from various sources.

**Contents:**
- Imaging sonar data
- Side-scan sonar
- Multibeam sonar
- Various underwater environments

**Use Cases:**
- Sonar SLAM development
- Object detection and classification
- Acoustic image processing

**Annotations:** Varies by dataset

### MIMIR-UW

**[MIMIR-UW](https://github.com/remaro-network/MIMIR-UW)** - A Multipurpose Synthetic Dataset for Underwater Navigation and Inspection

**Type:** Synthetic dataset (simulation-generated)

**Contents:**
- Synthetic underwater imagery
- Sonar data
- Ground truth poses
- Various underwater scenarios

**Advantages:**
- Ground truth available for synthetic scenes
- Controllable conditions
- Large-scale data generation
- Multiple sensor modalities

**Use Cases:**
- Training deep learning models
- Algorithm validation
- Simulation-to-real transfer studies

## Object Detection Datasets

### Underwater Trash Detection

**[LearnOpenCV Underwater Trash Dataset](https://learnopencv.com/yolov6-custom-dataset-training/)**

**Contents:**
- Underwater images with trash
- Bounding box annotations
- Various marine environments

**Use Cases:**
- Object detection training
- Marine conservation applications
- Environmental monitoring

**Format:** Images with YOLO-format annotations

### Marine Structure Datasets

Datasets for inspecting underwater structures:

**Common Targets:**
- Pipelines
- Cables
- Ship hulls
- Offshore platforms
- Underwater infrastructure

**Characteristics:**
- Often proprietary or restricted
- Some academic releases available
- Competition datasets (e.g., from MBZIRC)

## Visual Odometry and SLAM Datasets

### Marine Visual Datasets

Datasets for visual navigation in underwater environments:

**Challenges:**
- Limited visibility
- Color distortion
- Artificial lighting
- Feature-sparse environments

**Available Datasets:**
- Research institution releases
- Competition datasets
- Simulated environments

## Surface Vehicle Datasets

### Maritime Survey Data

Datasets from autonomous surface vehicles:

**Contents:**
- GNSS/GPS tracks
- Radar data
- Camera imagery
- Meteorological data
- Wave conditions

**Sources:**
- Research projects
- Maritime competitions (VRX)
- Academic institutions

## Simulation-Generated Datasets

### Gazebo-Based Datasets

Data generated from maritime simulators:

**Sources:**
- VRX competition scenarios
- DAVE simulation
- Custom simulation environments

**Advantages:**
- Perfect ground truth
- Reproducibility
- Controllable conditions
- Unlimited data generation

**Use Cases:**
- Initial algorithm development
- Sim-to-real transfer research
- Benchmarking

## Acoustic Communication Datasets

Underwater acoustic channel data:

**Contents:**
- Channel impulse responses
- Communication performance metrics
- Environmental parameters

**Sources:**
- Oceanographic institutions
- Research projects
- ns-3 UAN simulations

## Environmental Datasets

### Oceanographic Data

Publicly available ocean data:

**Sources:**
- NOAA databases
- Copernicus Marine Service
- World Ocean Database
- Argo float data

**Contents:**
- Temperature and salinity profiles
- Current measurements
- Wave conditions
- Bathymetric data

**Use Cases:**
- Environment modeling
- Mission planning
- Algorithm validation

## Dataset Repositories

### General Repositories

* **IEEE DataPort:** Hosts various maritime robotics datasets
* **Kaggle:** User-contributed datasets including underwater imagery
* **Academic Institutions:** Many labs publish datasets with papers

### MGDS (Marine Geoscience Data System)

MGDS provides public marine geoscience datasets and archives with broad coverage and consistent metadata practices.

**Data Sources:**

* **[MGDS Data Portal](https://www.marine-geo.org/):** Searchable marine geoscience datasets
* **[MGDS Search](https://www.marine-geo.org/tools/search/):** Dataset listings and metadata

**Access:**
- Public datasets with downloads and metadata
- DOI-backed dataset references where available
- Publications often link to related datasets

**Why MGDS Data Matters:**
- Publicly archived data with traceable provenance
- Useful for validating mapping, navigation, and perception pipelines

### Competition Datasets

Some maritime robotics competitions release datasets or logs; check individual competition sites and repositories for availability and licensing (for example, VRX simulation logs).

## Creating and Sharing Datasets

### Best Practices

When creating datasets for the community:

1. **Documentation:**
   - Sensor specifications
   - Calibration parameters
   - Environmental conditions
   - Data collection procedures

2. **Formats:**
   - ROS bags for multi-sensor data
   - Standard formats (images, point clouds)
   - Metadata in human-readable format (YAML, JSON)

3. **Annotations:**
   - Ground truth when available
   - Annotation methodology
   - Quality assessment

4. **Licensing:**
   - Clear license (Creative Commons, MIT, etc.)
   - Citation requirements
   - Usage restrictions if any

5. **Accessibility:**
   - Permanent hosting (university servers, Zenodo, etc.)
   - DOI for citation
   - README with quick-start guide

### Contributing to This List

If you have a maritime robotics dataset to share:

1. Add it to this page via pull request
2. Include: name, link, brief description, use cases
3. Specify data type and format
4. Indicate if annotations/ground truth are available
5. Provide license information

## Dataset Comparison

| Dataset | Type | Sensors | Size | Ground Truth | License | Format |
|---------|------|---------|------|--------------|---------|--------|
| REMARO Data | Multi-sensor | Various | Varies | Partial | Open | ROS bags |
| OpenSonarDatasets | Sonar | Imaging/Side-scan | Varies | Partial | Open | Mixed |
| MIMIR-UW | Synthetic | Camera/Sonar | Large | Full | Open | Simulation |
| Trash Detection | Vision | Camera | Medium | Bounding boxes | Open | Images |

---

*This page was last updated: {{ git_revision_date_localized }}*

--8<-- "docs/goatcounter.html"
