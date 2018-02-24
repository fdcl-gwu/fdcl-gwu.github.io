from github import Github

import os
import pdb

# !!! DO NOT EVER USE HARD-CODED VALUES HERE !!!
# Instead, set and test environment variables, see README for info
GH_ACCSS_TKN = os.environ['GH_ACCSS_TKN']
g = Github(GH_ACCSS_TKN)


def write_all_repos(fdcl, member_repos):

    with open('repos_all.md', 'w') as f:
        f.write('# All Repositories\n')
        f.write('All the repositories in FDCL in chronological order\n\n')
        f.write('Repository | Collaborators | Description\n')
        f.write('---- | ---- | ----\n')

        for repo in fdcl.get_repos():
            print(repo.name)
            f.write('[{}]({}) | '.format(repo.name, repo.html_url))
            for mem in repo.get_contributors():
                if mem.login in member_repos['members']:
                    f.write('[{}](repo_member.md#{})<br/>'.format(mem.name,
                            mem.login))

            f.write(' | {}\n'.format(repo.description))


for org in g.get_user().get_orgs():
    if org.login == 'fdcl-gwu':
        fdcl = org
        break

private_repos = []
public_repos = []
member_repos = {}
member_repos['members'] = []

for mem in fdcl.get_members():
    # print(mem.login)
    member_repos[mem.login] = {}
    member_repos[mem.login]['repos'] = []
    member_repos[mem.login]['name'] = mem.name
    member_repos[mem.login]['url'] = mem.html_url
    member_repos[mem.login]['email'] = mem.email
    member_repos[mem.login]['avatar'] = mem.avatar_url
    member_repos['members'].append(mem.login)

for repo in fdcl.get_repos():
    break
    if repo.private:
        private_repos.append(repo)
    else:
        public_repos.append(repo)

    for mem in repo.get_contributors():
        if mem.login in member_repos['members']:
            member_repos[mem.login]['repos'].append(repo)

write_all_repos(fdcl, member_repos)



# repo.get_tags
# repo.description
# repo.html_url
# repo.get_contributors # list
# repo.get_releases
# repo.pushed_at # datetime
# repo.get_tags # list
