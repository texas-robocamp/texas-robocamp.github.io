# Texas RoboCamp Website

Home to information about the camp as well as all materials and tutorials.

Based on the [Jekyll Documentation Theme](https://github.com/tomjoht/documentation-theme-jekyll).

## Development

### Installation

Install ruby and bundler.

    bundle install

### Viewing Locally

Then

    bundle exec jekyll serve

### Adding a Tutorial

Tutorials are kept under the tutorial folder. Be sure to add the correct frontmatter to each markdown file. Example:

    ---
    title: "Using Linux"
    tags: [linux]
    keywords: linux
    last_updated: July 2, 2018
    summary: ""
    sidebar: tutorials
    permalink: using_linux.html
    ---

Then insert the tutorial into the sidebar by editing the `_data/sidebars/tutorials.yaml` file appropriately.