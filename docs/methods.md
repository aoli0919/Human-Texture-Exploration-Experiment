# Analysis methods and reproducibility scope

## Scope

`python -m human_texture reproduce --data-dir data --output-dir outputs/reproduction` validates and summarizes the two processed workbooks, checks referenced raw-force files by default, and performs a defined participant-level association analysis. It does not operate hardware or process videos. These are reproducible analyses of supplied data, without a claim to recover a paper's tables, figures, R² values, or inferential tests.

## Extraction and validation

The inputs are `Sheet1` from each workbook in `data/processed/`. Empty sheets and trailing blank columns are excluded. The ratings workbook supplies material labels, question codes, scale directions, and participant columns. Its 50 × 40 matrix is compared with the corresponding matrix in `All in one.xlsx` before assembling 400 participant–texture observations.

Force text supplies each session folder, raw-force filename, and reported average force. The analysis parses this numeric force and checks it against the NF cache. SS is reconstructed from supplied unrounded Speed values using two-decimal decimal `ROUND_HALF_UP`. These are extraction and rounding operations, with no normalization. Original workbooks are not recalculated or overwritten.

Validation checks expected participant/texture coverage, numeric rating bounds, cross-workbook agreement, and source references. By default, it also checks the 400 raw-force files; `--skip-raw` omits that step. Raw quality checks establish structure, timestamp coverage, and numeric validity, not the historical derivation of reported force. Exact source ranges and definitions are in the [data dictionary](data_dictionary.md).

## Descriptive statistics and figures

Rating summaries group by texture and dimension, using 40 participant ratings per combination. Exploration summaries group reported force and rounded reported speed by texture. Distribution statistics accompany the plots. Force/speed labels state that physical units are unspecified.

The rating heatmap shows texture–dimension means. Force/speed plots show variation across participants. Coarseness retains the original direction, −10 fine and +10 coarse, despite the source question's Fineness label. Temperature denotes perceived warmth/coldness rather than physical degrees.

## Within-participant Spearman analysis

For each of 40 participants, the analysis compares ratings across ten textures with that participant's reported exploration values. Five dimensions × two exploration measures produce up to 400 participant-level Spearman coefficients.

Spearman rho is the Pearson correlation of ranks, with average ranks for ties. Constant inputs have undefined correlation and remain unavailable rather than becoming zero. Each rho uses ten texture pairs, not forty participants. The summary table and figure describe the participant-level rho distribution for each dimension–measure pair and count defined coefficients.

This treats participants as repeated-measurement units rather than treating all 400 observations as independent subjects. It reports descriptive associations, without pooled p-values, causal claims, or an assertion that it reproduces a paper's regression model. The shared set of ten materials limits extrapolation to other textures.

## Missing original transformations

The workbooks do not specify physical units for force/speed, force component selection, contact segmentation, baseline correction, filtering, or exclusion rules. Historical collection scripts record sensor channels and video; they do not implement those transformations.

A read-only audit of three raw-force files found that their stored reported force did not match either full-record mean absolute Fz or full-record mean resultant force. Those candidate calculations are diagnostics, not replacements for the missing method. A guessed formula would define a different measurement; raw-force-to-NF reconstruction remains unverified.

The author confirms that speed processing included pixel-to-millimetre spatial calibration. The coefficient and calibration files are no longer available. The final speed time unit and tracking/time conversion implementation remain unconfirmed. Values are preserved as reported, without assuming mm/s. Exact video-to-Speed reconstruction is currently unavailable.

Material numbering is explicit in questionnaire groups and source force filenames. Aligning named materials with force/speed rows relies on shared workbook ordering and identical rating matrices; an independent historical plate-number mapping is not established here.

Raw checks improve file traceability and quality reporting but cannot resolve these gaps. A run with `--skip-raw` additionally omits raw-file validation and supports processed-data analysis only.

## Rerunning

From the repository root, install with `python -m pip install -e .`. Run `verify --data-dir data` to inspect inputs and `reproduce` to generate tables, figures, `audit.json`, and `report.md`. Keep source workbooks and raw files unchanged when comparing runs. The report and audit provide run-specific validation evidence.

New camera/sensor acquisition is separate; see [acquisition.md](acquisition.md). A dry-run acquires no data, and successful processed-data analysis does not validate physical devices.
