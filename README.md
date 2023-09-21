# Useful Information for the Authors

## Steps to follow when developing a new feature/function:
- Step 01: Create an issue for the feature and link an existing branch or create a new branch. Draft a proposal for the feature (specify inputs and outputs etc.) in the issue and discuss with the team.
- Step 02: Develop the code while regularly commiting the changes to the remote repository. Try to log important milestones and decisions during the development process in the respective issue as much as possible. If necessary, you could use Matlab built-in tools such as [Matlab Profiler](https://www.mathworks.com/help/matlab/matlab_prog/profiling-for-improving-performance.html) and/or [Matlab Code Analyzer](https://www.mathworks.com/help/matlab/matlab_prog/matlab-code-analyzer-report.html) to evaluate your code. If you do so, make sure to attach these reports in the issue.
- Step 03: Once completed, change the assignee of the corresponding issue to **Hoang** to check whether the agreed upon naming convensions are met.
- Step 04: **Hoang** will assign the issue back to the author and he/she should create one or more demo files to showcase the basic functionality of the feature.
- Step 05: Create a documents using a Github .md files that contain information about the feature and its demo files. Place them in the respective folders.
- Step 06: Assign the issue to a third person to run the demo files and evaluate your work. This person is responsible to make sure that the feature is working as intended and that the provided documentation is sufficient to use it without consulting the author. Preferably this should be done by someone with a different field of expertise in order to ensure the clarity of the documentation.
- Step 07: Create a pull request and assign the issue to **Dhanika** to merge the branch to the main repository.

## Folder structure for a class:
Please follow the following folder structure as a guideline when creating a new class:  
  
[class name] folder
  - 'demo' folder
    - [class name]_demo1.m
    - [class name]_demo2.m
  - [class name].m file
  - README.md file
