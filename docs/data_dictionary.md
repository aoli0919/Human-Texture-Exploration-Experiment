# Data dictionary

Paths are relative to the repository root. The analysis reads the processed workbooks and, by default, raw-force CSVs. It does not read questionnaire exports, videos, or participant documents.

## Processed workbooks

Both workbooks in `data/processed/` contain `Sheet1`, `Sheet2`, and `Sheet3`. Only `Sheet1` is populated; import it explicitly.

### Perceptual Dimension Ratings.xlsx

The populated range is `A1:AQ51`; the declared extent also includes blank column AR. Row 1 is the header. Rows 2–51 are 50 texture/question combinations with 40 numeric participant responses: 2,000 ratings, no missing values, all within −10 to +10 in the audited source.

| Column | Header | Meaning |
| --- | --- | --- |
| A | `No.` | Question code `Q<texture>_<dimension>` |
| B | `Texture` | Material name |
| C | `Question` | Original question and scale endpoints |
| D:AQ | `Participant #1` … `Participant #40` | Numbered participant ratings |

### All in one.xlsx

The populated range is `A1:AO91`; the declared extent also includes blank column AP. Column A holds labels; B:AO hold `Participant#1` … `Participant#40`. Its first 50 data rows exactly match the 2,000 ratings in the other workbook, in the same order.

| Rows | Labels | Contents |
| --- | --- | --- |
| 2–51 | `T`, `R`, `F`, `H`, `T*`, repeated ten times | Five dimensions for each texture |
| 52–61 | `NF1` … `NF10` | Formulas extracting `Average Force` from rows 72–81 |
| 62–71 | `SS1` … `SS10` | Formulas rounding rows 82–91 to two decimals |
| 72–81 | `Force1` … `Force10` | Text containing session folder, raw-force filename, stored average force |
| 82–91 | `Speed1` … `Speed10` | Numeric speed before two-decimal rounding |

All 800 formula caches are populated: 400 NF values agree with parsed force text and 400 SS values agree with rounded Speed. No participant-block cells are missing. `NF` is not established to mean “normalized force.”

Force text has the form `Folder: <session>, File: force_data_<texture>.csv, Average Force: <value>`. Strip trailing whitespace before parsing. All 400 references resolve: each participant maps to one session, with 40 sessions and ten files per session. Session labels do not establish presentation order or demographics.

## Texture and dimension codes

| Texture/question group | Material |
| --- | --- |
| 1 | Felt |
| 2 | Mesh |
| 3 | Nylon |
| 4 | Acrylic |
| 5 | Fashion Fabric |
| 6 | Fur |
| 7 | Canvas |
| 8 | Wool |
| 9 | Cotton |
| 10 | Wood |

Material names are explicit in the ratings workbook. Joining them to numbered force/speed rows uses the identical rating matrices and shared row order. No independent historical plate-number registry has been established.

| Question suffix | Combined label | Analysis field | −10 | +10 |
| --- | --- | --- | --- | --- |
| 1 | `T` | `traction` | Very slippery | Very sticky |
| 2 | `R` | `roughness` | Very smooth | Very rough |
| 3 | `F` | `coarseness` | Very fine | Very coarse |
| 4 | `H` | `hardness` | Very soft | Very hard |
| 5 | `T*` | `temperature` | Very cold | Very hot |

The source calls question 3 “Fineness,” but positive values mean coarse. `coarseness` preserves recorded values without a sign reversal. Temperature is perceived sensation, not measured surface temperature.

## Generated observations.csv

Each row represents one participant–texture combination: 40 × 10 = 400 rows. Participant numbers identify dataset columns, not identity or collection order.

| Field | Meaning / units |
| --- | --- |
| `participant_id` | Participant column identifier, `P01`–`P40` |
| `session_id` | Folder parsed from force text |
| `texture_id` | Texture number, 1–10 |
| `texture` | Material from ratings workbook |
| `traction`, `roughness`, `coarseness`, `hardness`, `temperature` | Original subjective ratings, −10 to +10 |
| `reported_force` | Parsed stored average force; physical units unspecified |
| `reported_speed` | Supplied Speed rounded to two decimals using decimal half-up rounding; units unspecified |
| `speed_unrounded` | Supplied Speed before rounding; units unspecified |
| `source_force_csv` | Corresponding raw-force CSV reference |

Reported force is already rounded in its source text; higher precision cannot be recovered from it. Force component, contact segmentation, baseline correction, filtering, and exclusion rules are not documented. The author confirms pixel-to-millimetre spatial calibration for speed, but the factor and files are no longer available, and the final speed time unit is unconfirmed. Do not infer mm/s solely from spatial calibration.

## Raw force

`data/raw/force/<session>/force_data_<texture>.csv` contains 400 files across 40 directories, each with ten numbered files. CSV headers:

```text
Fx,Fy,Fz,Tx,Ty,Tz,Timestamp
```

Fx/Fy/Fz are recorded force channels; Tx/Ty/Tz are torque channels. The files do not declare units or a sensor-to-surface coordinate frame. The legacy decoder divides force integers by 100 and torque integers by 1000; these factors alone are not a complete unit/calibration declaration. `Timestamp` is produced by Python `time.time()`: wall-clock seconds since the Unix epoch, recorded after a sensor read.

The suffix is the displayed texture label, selected in randomized order, not the trial's ordinal presentation position. Historical CSVs do not supply a complete trial-order log. Calculate sampling intervals from actual timestamps; camera FPS is not force sampling frequency.

## Other original materials

`data/raw/video/` holds the original videos; `data/raw/questionnaire/` holds the original questionnaire export. Neither is needed by the analysis. Original participant information and consent PDFs remain under `docs/`; their presence does not establish a separately supplied ethical approval document. The reproduction command does not inspect them.
