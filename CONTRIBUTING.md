# Contributing Guide

Thank you for your interest in the ROS2 Robot Navigation and Exploration System! We welcome various forms of contributions, including but not limited to:

- 🐛 Bug reports
- 💡 Feature suggestions
- 📝 Documentation improvements
- 🔧 Code fixes
- 🧪 Test cases
- 🌍 Translations

## 🚀 Quick Start

### 1. Fork and Clone Repository

```bash
# Fork this repository to your GitHub account
# Then clone your fork
git clone https://github.com/yourusername/ros2-smart-exploration.git
cd ros2-smart-exploration

# Add upstream repository
git remote add upstream https://github.com/originalowner/ros2-smart-exploration.git
```

### 2. Set Up Development Environment

```bash
# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# or
venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt

# Install development dependencies
pip install -r requirements-dev.txt
```

### 3. Create Branch

```bash
# Create new branch from main
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

## 📝 Development Standards

### Code Style

We use the following tools to maintain code quality:

```bash
# Code formatting
black first_pkg/
isort first_pkg/

# Code checking
flake8 first_pkg/
pylint first_pkg/

# Type checking
mypy first_pkg/
```

### Commit Message Standards

We use [Conventional Commits](https://www.conventionalcommits.org/) specification:

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

**Types include:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation update
- `style`: Code style changes
- `refactor`: Code refactoring
- `test`: Adding tests
- `chore`: Build process or auxiliary tool changes

**Example:**
```
feat(exploration): Add RRT exploration strategy

- Implement boundary point evaluation exploration algorithm
- Add exploration efficiency calculation functionality
- Optimize target point selection logic

Closes #123
```

### Testing Requirements

Before submitting code, please ensure:

```bash
# Run all tests
colcon test --packages-select first_pkg

# Run specific tests
python -m pytest tests/test_exploration.py -v

# Check test coverage
python -m pytest --cov=first_pkg tests/
```

## 🔄 Submission Process

### 1. Submit Code

```bash
# Add changes
git add .

# Commit changes
git commit -m "feat: Add new feature"

# Push to your fork
git push origin feature/your-feature-name
```

### 2. Create Pull Request

1. Visit your GitHub fork page
2. Click "New Pull Request" button
3. Select correct branch
4. Fill in PR description, including:
   - Brief description of changes
   - Problems solved or features added
   - Testing status
   - Related Issue number

### 3. PR Template

```markdown
## Change Description
Briefly describe the changes in this PR

## Change Type
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Code refactoring
- [ ] Performance optimization
- [ ] Other

## Testing Status
- [ ] Unit tests added
- [ ] Integration tests performed
- [ ] Gazebo simulation tested
- [ ] All tests passed

## Related Issue
Closes #(issue number)

## Screenshots/Videos
If necessary, add screenshots or demo videos

## Checklist
- [ ] Code follows project style guidelines
- [ ] Self code review performed
- [ ] Necessary comments added
- [ ] Documentation updated accordingly
- [ ] Changes don't produce new warnings
```

## 🐛 Bug Reports

### Bug Report Template

```markdown
**Bug Description**
Clearly and concisely describe the bug

**Reproduction Steps**
1. Go to environment '...'
2. Click on '....'
3. Scroll down to '....'
4. See error

**Expected Behavior**
Clearly and concisely describe what you expected to happen

**Actual Behavior**
Clearly and concisely describe what actually happened

**Environment Information**
- OS: [e.g. Ubuntu 22.04]
- ROS2 Version: [e.g. Humble]
- Python Version: [e.g. 3.8]
- Hardware: [e.g. Simulation/Real Robot]

**Additional Information**
Add any other context about the problem

**Log Files**
If applicable, add relevant log files
```

## 💡 Feature Requests

### Feature Request Template

```markdown
**Feature Description**
Clearly and concisely describe the feature you'd like to see

**Problem Background**
Describe what problem this feature would solve

**Proposed Solution**
Clearly and concisely describe how you'd like to implement this feature

**Alternatives**
Clearly and concisely describe any alternative solutions or features you've considered

**Additional Information**
Add any other context or screenshots about the feature request
```

## 📚 Documentation Contributions

### Documentation Types
- API documentation
- Tutorials and examples
- Configuration guides
- Troubleshooting guides
- Translations

### Documentation Standards
- Use Markdown format
- Include code examples
- Add necessary screenshots
- Keep content updated

## 🏷️ Label Guide

We use the following labels to organize Issues and PRs:

**Type Labels:**
- `bug`: Issues that need fixing
- `enhancement`: New features or improvements
- `documentation`: Documentation related
- `question`: Issues that need more information

**Priority Labels:**
- `priority: high`: High priority
- `priority: medium`: Medium priority
- `priority: low`: Low priority

**Status Labels:**
- `status: needs-triage`: Needs triage
- `status: in-progress`: In progress
- `status: blocked`: Blocked
- `status: needs-review`: Needs review

## 🤝 Community Code of Conduct

### Our Commitment

To foster an open and welcoming environment, we commit to:

- Respect all contributors
- Accept constructive criticism
- Focus on what's best for the community
- Show empathy towards other community members

### Unacceptable Behavior

- Use of sexualized language or imagery
- Personal attacks or political attacks
- Public or private harassment
- Publishing others' private information without permission
- Other conduct inappropriate in a professional environment

## 📞 Getting Help

If you encounter problems during contribution:

1. Check [Issues](https://github.com/originalowner/ros2-smart-exploration/issues) for similar problems
2. Ask questions in [Discussions](https://github.com/originalowner/ros2-smart-exploration/discussions)
3. Contact maintainers

## 🙏 Acknowledgments

Thank you to all developers who have contributed to this project! Your contributions make this project better.

---

**Note:** By participating in this project, you agree to abide by this Code of Conduct. Project maintainers have the right to remove, edit, or reject comments, commits, code, wiki edits, issues, and other contributions that do not comply with this Code of Conduct.
