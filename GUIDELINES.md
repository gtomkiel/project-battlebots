# Project guidelines

## Table of contents
1. [General](GUIDELINES.md#general)
2. [GIT](GUIDELINES.md#git)

## General 
Indentation: 3 spaces<br>
Charset: UTF-8

Use camel case when writing code and naming files<br><br>
For naming variables, functions, files etc. use camel case

Some camelCase examples: 
* $exampleVariable = "foo";
* functionFooBar();
* exampleFileName.php

To be continued...

## Git
### Contribution
#### Branches
Each issue should have their own separate branch. 
  
Name of the branch should be: ```issue[github issue]```  
Example would be ```issue01``` for issue #1  

#### Commits
Only rule for the commits is to put a short, accurate description of the changes.

#### Pull requests
Pushing your commits directly to the main branch is locked.
  
In order to contribute you need to create the pull request:  
* Make sure to include the name of the issue in title and put a description.  
* Request @gtomkiel for review of the pull request.   
* Create the pull request only when you are done with the specific issue.  
* Don't combine multiple issues in a single request.

If the request is accepted the code will be pulled into the main branch.   

### Quick guide
#### Setting up local repository
1. Clone the repository
```
git clone git@github.com:gtomkiel/project-battlebots.git
```
2. Navigate to the folder where you have the project files
```
cd project-battlebots
```
3. Switch to your branch
```
git checkout <branch-name>
```
4. (Optional) Make sure that your branch is up to date
```
git pull
```

#### Commiting your changes to github
1. Add files to commit
```
git add .
```
2. Commit your changes locally (Make sure to put descriptive messeage!)
```
git commit -m "<your-messeage>"
```
3. Make sure that your branch is up to date
```
git pull
```
4. Push your changes to GitHub
```
git push
```

#### Reverting your push
1. Find the hash of you commit from GitHub
2. Revert your changes
```
git revert <commit-hash>
```
3. Commit your changes
```
git commit -m "<your-messeage>"
```
4. Push your changes
```
git push
```

#### Sync your branch to main
1. Make sure you are in your branch
```
git checkout <branch-name>
```
```
git fetch origin
git merge origin/main
```
5. (Optional) Fix possible merge conflicts

#### To create a pull request
1. Go to the GitHub repository
2. Click pull requests
3. Make a new pull request
4. Click on your branch
5. Write a title and short description
6. Wait for feedback
