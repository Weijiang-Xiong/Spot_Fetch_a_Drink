# Spot_Fetch_a_Drink

This is the repository for Project Spot Fetch a Drink at Aalto University.

In this project, we are going to develop a voice command functionality for the [Spot Robot](https://www.bostondynamics.com/spot).
Upon project completion, Spot would become a nice life assistant, which is able to take voice command from you and perform tasks, like fetching an object and going somewhere.
The applications of this project includes routine inspection and damage fixing in a large factory plant, autonomous guiding for blind people and daily care for the elderly/children or disabled people.

## Tips for setting up your branch 

Get this repo and create your own branch.

- Open a terminal where you wish to store the files

- Clone the repo with `git clone https://github.com/Weijiang-Xiong/Spot_Fetch_a_Drink.git Spot_Masters`

- Navigate to the folder `cd Spot_Masters`

- Create your branch `git checkout -b your_branch_name`

- Push to remote and create a remote branch `git push origin your_branch_name`

- Keep track of the remote branch you just created `git checkout -b your_branch_name --track <remote>/your_branch_name`

Then you are on your own branch, and you can work on your own module, say create some new functionality for the project.

If you want to work on a sub-branch of your branch, like develop some new features based on your module.

- Create a new branch based on your existing branch `git checkout -b some-feature your_branch_name`

- When you have done this new feature, add the new files or modified files to git `git add path_to_files`

- commit the change `git commit -m "developed a feature to do something"`

- Merge the new feature to your branch (merging branch to main branch is similar)
``` 
git pull origin your_branch_name
git checkout your_branch_name
git merge --no-ff some-feature
git push origin your_branch_name
```

