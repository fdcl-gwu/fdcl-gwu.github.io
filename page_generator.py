from github import Github

import os
import pdb

GH_ACCSS_TKN = os.environ['GH_ACCSS_TKN']
g = Github(GH_ACCSS_TKN)

for org in g.get_user().get_orgs():
    if org.login == 'fdcl-gwu':
        fdcl = org
        break

private_repos = []
public_repos = []
member_repos = {}
member_repos['members'] = []

for mem in fdcl.get_members():
    print(mem.login)
    member_repos[mem.login] = {}
    member_repos[mem.login]['repos'] = []
    member_repos[mem.login]['name'] = mem.name
    member_repos[mem.login]['url'] = mem.html_url
    member_repos[mem.login]['email'] = mem.email
    member_repos[mem.login]['avatar'] = mem.avatar_url
    member_repos['members'].append(mem.login)

for repo in fdcl.get_repos():
    if repo.private:
        private_repos.append(repo)
    else:
        public_repos.append(repo)

    for mem in repo.get_contributors():
        if mem.login in member_repos['members']:
            member_repos[mem.login]['repos'].append(repo)

pdb.set_trace()

# repo.get_tags
# repo.description
# repo.html_url
# repo.get_contributors # list
# repo.get_releases
# repo.pushed_at # datetime
# repo.get_tags # list
