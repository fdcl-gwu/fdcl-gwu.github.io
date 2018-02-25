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
            f.write('[{}]({}) | '.format(repo.name, repo.html_url))
            for mem in repo.get_contributors():
                if mem.login in member_repos['members']:
                    f.write('[{}](repos_member#{})<br/>'.format(mem.name,
                            mem.login))

            f.write(' | {}\n'.format(repo.description))


def write_repos(repos):
    with open('repos_member.md', 'a') as f:
        f.write('<p>&#160;<br></p>\n')
        f.write('\n\n<a name="{}"></a>\n'.format(repos['login']))
        f.write('\n<hr>\n')
        f.write('<p>&#160;<br></p>\n\n')
        f.write('<img src="{}"  width="200"/>\n'.format(repos['avatar']))
        f.write('## {}\n  '.format(repos['name']))
        f.write('Personal profile: [{}]({})  \n'.format(repos['login'],
                repos['url']))
        f.write('Email: {}  \n'.format(repos['email']))

        f.write('<p>&#160;<br></p>\n\n')
        if not len(repos['repos']) == 0:
            f.write('Repository | Description\n')
            f.write('---- | ---- \n')

        for repo in repos['repos']:
            f.write('[{}]({}) | '.format(repo.name, repo.html_url))
            f.write(' {}\n'.format(repo.description))
        f.write('\n\n')


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
    member_repos[mem.login]['login'] = mem.login
    member_repos[mem.login]['name'] = mem.name
    member_repos[mem.login]['url'] = mem.html_url
    member_repos[mem.login]['email'] = mem.email
    member_repos[mem.login]['avatar'] = mem.avatar_url
    member_repos['members'].append(mem.login)

for repo in fdcl.get_repos():
    print(repo.name)

    if repo.private:
        private_repos.append(repo)
    else:
        public_repos.append(repo)

    for mem in repo.get_contributors():
        if mem.login in member_repos['members']:
            member_repos[mem.login]['repos'].append(repo)

f = open('repos_member.md', 'w')
f.close()

write_all_repos(fdcl, member_repos)

for mem in member_repos['members']:
    write_repos(member_repos[mem])


# repo.get_tags
# repo.description
# repo.html_url
# repo.get_contributors # list
# repo.get_releases
# repo.pushed_at # datetime
# repo.get_tags # list
