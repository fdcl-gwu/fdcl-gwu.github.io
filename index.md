---
layout: default
---

This page is generated using [Jekyll](http://jekyllrb.com/): a static page website generator.

[Github Pages](https://pages.github.com/) can automatically host websites as well.
This allows for relatively simple static pages, which are written in [Markdown](https://daringfireball.net/projects/markdown/), to be automatically generated and hosted online.

The following sections are automatically generated using Jekyll.

## Work

Our public repositories are:

{% for repository in site.github.public_repositories %}
  * [{{ repository.name | replace:'_','-' }}]({{ repository.html_url }}) - {{ repository.description }}
{% endfor %}


## Members of FDCL
<!-- [{{ site.github.owner_name }}]({{ site.github.owner_url }}) -->

<!-- Member | Description   -->
{% for member in site.github.organization_members %}
  <img src="{{member.avatar_url}}" width="60"> [{{member.login}}]({{member.html_url}})
{% endfor %}
