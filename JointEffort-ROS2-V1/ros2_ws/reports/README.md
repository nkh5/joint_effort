# Model data and test results

The bundle keeps the workspace-study data needed by the hover tools, collision data, the proposed motor map, and a small set of test results.

Run the check tools to produce new results and logs. Mock tests do not establish physical accuracy, load capacity, or stopping distance.

`iron/workspace-study.json` contains proposed hover endpoints and their modeled collision results. Changes to geometry or limits require a new study and collision checks.

`iron/proposed-motor-map.csv` contains unconfirmed settings. It is not a commissioned motor map.
