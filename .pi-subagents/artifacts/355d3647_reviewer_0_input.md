# Task for reviewer

[Read from: D:\dev\Robot\小车\src\plan.md, D:\dev\Robot\小车\src\progress.md]

在仓库 D:/dev/Robot/小车/src 中只读审查 pathfinder_basic_route/pathfinder_basic_route.ino，并与 pathfinder_commented/pathfinder_commented.ino 对比。用户现象是 basic_route 上传后“小车不开跑”，而 commented 的四光电路线正常。请重点找会导致启动阶段完全不动的确定性/高概率原因，按优先级列出，给出文件行号和触发条件；区分启动阻塞、标定失败、安全停车、电机极性、路线逻辑问题。不要修改任何文件。

## Acceptance Contract
Acceptance level: attested
Completion is not accepted from prose alone. End with a structured acceptance report.

Criteria:
- criterion-1: Return concrete findings with file paths and severity when applicable

Required evidence: review-findings, residual-risks

Finish with a fenced JSON block tagged `acceptance-report` in this shape:
Use empty arrays when no items apply; array fields contain strings unless object entries are shown.
`criteriaSatisfied[].status` must be exactly one of: satisfied, not-satisfied, not-applicable.
`commandsRun[].result` must be exactly one of: passed, failed, not-run.
`manualNotes` and `notes` are optional strings; an empty string means no note and does not satisfy `manual-notes` evidence.
```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "specific proof"
    }
  ],
  "changedFiles": [
    "src/file.ts"
  ],
  "testsAddedOrUpdated": [
    "test/file.test.ts"
  ],
  "commandsRun": [
    {
      "command": "command",
      "result": "passed",
      "summary": "short result"
    }
  ],
  "validationOutput": [
    "validation output or concise summary"
  ],
  "residualRisks": [
    "none"
  ],
  "noStagedFiles": true,
  "diffSummary": "short description of the diff",
  "reviewFindings": [
    "blocker: file.ts:12 - issue found, or no blockers"
  ],
  "manualNotes": "anything else the parent should know"
}
```