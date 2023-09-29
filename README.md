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

## Documentation templates:
- Class documentation
  When creating the documentation of a class use [this](templates-do_not_publish/class_doc.md) template.
- Demo documentation
  When creating the documentation of a demo use [this](templates-do_not_publish/demo_doc.md) template.

----
# FAQ

## What is Git?
A version control system helps manage your files and track the history of changes in your projects. A version control system makes it clear which changes to the project are made, who made those changes, and when those changes occurred. Version control also makes it easy to rewind to a previous version of your project if, for example, you discover a bug in your code and want to revert to a past version. 

**Git** is a widely-used version control system used to manage code. Git allows you to save drafts of your code so that you can look back at previous versions and potentially undo complicated errors. A project managed with Git is called a repository. A repository contains all of the files and folders associated with your project. It also includes the revision history of each file. The file history is a series of snapshots in time, known as commits. A commit tells Git that you made some changes which you want to record. When you make a commit in Git you will see “commit to main.” This is referring to the default branch, which can be thought of as the production-ready version of your project. 

The most straight forward way of interacting with Git is to use your command-line. You can have an overview of how you can use the command-line to interact with Git [here](https://aguaclara.github.io/aguaclara_tutorial/git-and-github/git-in-the-command-line.html). However, today there are ease-of-use interfaces that make it easy to interact with Git without using the command-line, known as [Git Clients](#what-is-a-git-client?).

## What is a Git Client?
