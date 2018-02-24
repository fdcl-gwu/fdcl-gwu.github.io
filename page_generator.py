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

# pdb.set_trace()
private_repos = []
public_repos = []
for repo in fdcl.get_repos():
    # print(repo)
    if repo.private:
        private_repos.append(repo)
    else:
        public_repos.append(repo)

pdb.set_trace()
# repo.html_url
# repo.get_contributors # list
# repo.get_releases
# repo.pushed_at # datetime
