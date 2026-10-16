# Directory migration

The repository now separates source data, runnable software, historical scripts, and generated results.

| Previous path | Current path |
| --- | --- |
| `Processed Data/` | `data/processed/` |
| `Raw Data/Raw_Force/` | `data/raw/force/` |
| `Raw Data/Raw_Video/` | `data/raw/video/` |
| `Raw Data/Raw_Questionaire/` | `data/raw/questionnaire/` |
| `Docs/` | `docs/` |
| `Processing Code/` | `legacy/acquisition/` |

All original workbooks, raw files, participant-document PDFs, and historical Python scripts retain their original bytes. The existing MIT license is unchanged. The original README is available in Git history; the current README describes the runnable workflow.

Use `src/human_texture/` for offline analysis and `acquisition/` for new collection sessions. The legacy scripts remain for provenance and are not the recommended entry point. Generated results live under ignored `outputs/` directories, with input SHA-256 hashes and software versions in each run's `audit.json`.
