from github import Github

import os
import pdb
# import urllib2

GH_ACCSS_TKN = os.environ['GH_ACCSS_TKN']
g = Github(GH_ACCSS_TKN)

for org in g.get_user().get_orgs():
    if org.login == 'fdcl-gwu':
        fdcl = org
        break

private_repos = []
public_repos = []
member_repo = {}
member_repo['members'] = []

for mem in fdcl.get_members():
    member_repo[mem.login] = {}
    member_repo[mem.login]['repos'] = []
    member_repo[mem.login]['name'] = mem.name
    member_repo[mem.login]['url'] = mem.html_url
    member_repo[mem.login]['email'] = mem.email
    member_repo[mem.login]['avatar'] = mem.avatar_url
    member_repo['members'] = mem.login

for repo in fdcl.get_repos():
    # print(repo)
    if repo.private:
        private_repos.append(repo)
    else:
        public_repos.append(repo)

    for mem in repo.get_contributors():
        if mem.login in member_repo['members']:
            member_repo[mem.login]['repos'].append(repo)

    if repo.name == 'fdcl-uav':
        break
pdb.set_trace()


# repo.get_tags
# repo.description
# repo.html_url
# repo.get_contributors # list
# repo.get_releases
# repo.pushed_at # datetime
# repo.get_tags # list
