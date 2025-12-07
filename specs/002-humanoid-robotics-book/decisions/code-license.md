# Architectural Decision: Code License Selection

**Date**: 2025-12-06
**Status**: Decided
**Decision Makers**: Book Author Team
**Related Tasks**: T014

## Context

All companion code repositories (12-15 repos) need a consistent open-source license that:
- Allows educational and commercial use without restrictions
- Is compatible with ROS 2 ecosystem (predominantly Apache 2.0)
- Is simple to understand for non-lawyers
- Enables community contributions and forks
- Supports bootcamp/corporate training use cases

## Options Considered

### Option 1: MIT License
**Type**: Permissive
**Complexity**: Very simple (~171 words)
**Commercial Use**: ✅ Allowed
**Attribution Required**: ✅ Yes
**Patent Grant**: ❌ No explicit patent grant

**Full Text (paraphrased)**:
- Permission to use, copy, modify, merge, publish, distribute, sublicense, sell
- Must include copyright notice and license in copies
- No warranty

**Pros**:
- ✅ **Simplicity**: Easiest license to understand (non-lawyers can read in 1 minute)
- ✅ **Permissive**: Minimal restrictions (allows proprietary derivatives)
- ✅ **Industry Standard**: Widely used in robotics (PyTorch, NumPy, many ROS packages)
- ✅ **No Copyleft**: Allows integration into proprietary products
- ✅ **Community Friendly**: Encourages forks, modifications, and contributions

**Cons**:
- ⚠️ **No Patent Grant**: Doesn't explicitly address patent rights (unlike Apache 2.0)
- ⚠️ **Less Protective**: Offers less legal protection than Apache 2.0

**Common Use**: PyTorch, Flask, Rails, many JavaScript libraries

### Option 2: Apache License 2.0
**Type**: Permissive
**Complexity**: Moderate (~8,600 words)
**Commercial Use**: ✅ Allowed
**Attribution Required**: ✅ Yes
**Patent Grant**: ✅ Explicit patent grant

**Key Terms**:
- Permission to use, modify, distribute (like MIT)
- **Explicit patent grant**: Contributors grant patent license
- **Patent retaliation**: If you sue for patent infringement, your license terminates
- Must provide NOTICE file documenting changes
- More detailed attribution requirements

**Pros**:
- ✅ **Patent Protection**: Explicit patent grant reduces litigation risk
- ✅ **ROS 2 Standard**: ROS 2 core and most packages use Apache 2.0
- ✅ **Enterprise Friendly**: Preferred by companies (Google, Apache Foundation)
- ✅ **Contribution Protection**: Contributor license agreement built-in

**Cons**:
- ⚠️ **Complexity**: 8,600-word license (intimidating for students)
- ⚠️ **NOTICE Requirements**: Must document changes in separate file (adds overhead)
- ⚠️ **Less Common in ML**: PyTorch, TensorFlow use different licenses

**Common Use**: ROS 2, Kubernetes, Android, Apache projects

### Option 3: GPL-3.0 (GNU General Public License v3)
**Type**: Copyleft (strong)
**Complexity**: Long (~5,600 words)
**Commercial Use**: ✅ Allowed (with restrictions)
**Attribution Required**: ✅ Yes
**Source Disclosure**: ✅ Required for derivatives

**Key Terms**:
- Derivative works **must** be released under GPL-3.0
- Source code must be made available
- Cannot integrate into proprietary software
- Patent grant included

**Pros**:
- ✅ **Copyleft**: Ensures derivatives remain open-source
- ✅ **Community Protection**: Prevents proprietary forks
- ✅ **Patent Grant**: Explicit patent protection

**Cons**:
- ❌ **Restrictive**: Cannot integrate into proprietary products
- ❌ **Commercial Friction**: Companies hesitant to use GPL code (contamination risk)
- ❌ **Ecosystem Mismatch**: Incompatible with Apache 2.0 (ROS 2 core)
- ❌ **Educational Barrier**: Students may want to use code in startup/commercial projects

**Decision**: **REJECTED** - Too restrictive for educational/commercial use

## Decision

**SELECTED: Option 1 - MIT License**

### Rationale

1. **Simplicity**: Students and educators can read and understand the entire license in < 1 minute
2. **Maximum Freedom**: Allows readers to use code in any context (academic, commercial, proprietary)
3. **Bootcamp/Corporate Training**: Companies can use book code in training programs without restrictions
4. **Community Contributions**: Permissive license encourages forks and community extensions
5. **Startup Friendly**: Students can incorporate book code into startup projects without legal concerns
6. **Industry Alignment**: Matches PyTorch, NumPy, and most ML/robotics Python packages
7. **Low Friction**: No NOTICE file requirements or change documentation overhead
8. **ROS 2 Compatible**: MIT is compatible with Apache 2.0 (ROS 2 core), can link without issues

### Why Not Apache 2.0?

While Apache 2.0 is the ROS 2 standard and offers better patent protection:
- **Complexity**: 8,600-word license vs. 171-word MIT license
- **Overhead**: NOTICE file requirements add friction for simple educational repos
- **Student Familiarity**: MIT is more recognizable in ML/AI community (PyTorch, Transformers)
- **Patent Risk**: Low for educational code examples (not production libraries)

**Compromise**: Include patent disclaimer in README.md:
> "This code is provided for educational purposes. For production use, consider consulting with legal counsel regarding patent implications."

### Why Not GPL-3.0?

GPL-3.0's copyleft would:
- Prevent students from using code in commercial projects
- Create friction with companies using book for training
- Be incompatible with Apache 2.0 (ROS 2 ecosystem)
- Reduce community contributions (companies hesitant to contribute to GPL projects)

## Implementation

### LICENSE File (all companion repositories)

```text
MIT License

Copyright (c) 2025 [Author Name]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### README.md Header (all repositories)

```markdown
## License

This code is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

You are free to:
- ✅ Use this code for educational, research, or commercial purposes
- ✅ Modify and distribute the code
- ✅ Integrate into proprietary projects

**Attribution**: If you use this code in publications or products, we appreciate (but do not require) a citation to:
> [Author Name]. (2025). *Physical AI & Humanoid Robotics: From Simulated Brains to Walking Bodies*. [Publisher].
```

### Book Content License (Separate)

**Book Text/Content**: Traditional copyright OR Creative Commons BY-NC-SA 4.0 (if dual-publishing web version)
**Code Examples**: MIT License (consistent with companion repositories)

## Repository Structure (12-15 repos)

All companion repositories include:
- `LICENSE` file (MIT License)
- `README.md` with license info and attribution suggestion
- `CONTRIBUTING.md` encouraging community contributions
- `.github/ISSUE_TEMPLATE/` for bug reports and feature requests

**Example Repositories**:
- `code/chapter-03-ros2-basics/` - MIT License
- `code/chapter-06-isaac-sim-intro/` - MIT License
- `code/chapter-14-vla-integration/` - MIT License
- `code/chapter-20-capstone/` - MIT License

## Consequences

### Positive
- ✅ Maximum accessibility for all readers (students, educators, companies)
- ✅ Encourages commercial adoption (startups can use code without legal review)
- ✅ Simple to understand (no legal expertise needed)
- ✅ Community-friendly (easy to fork and contribute)
- ✅ Compatible with ROS 2 ecosystem (can link with Apache 2.0 code)
- ✅ Matches PyTorch/ML ecosystem norms

### Negative
- ⚠️ No explicit patent grant (unlike Apache 2.0)
- ⚠️ Allows proprietary derivatives (some may prefer copyleft like GPL)
- ⚠️ Minimal legal protection for contributors

### Neutral
- ⚖️ Different from ROS 2 core (Apache 2.0), but compatible
- ⚖️ Patent disclaimer in README mitigates patent risk concerns

## Validation Against Requirements

- ✅ **Educational Use**: Unrestricted ✅
- ✅ **Commercial Use**: Unrestricted ✅
- ✅ **ROS 2 Compatibility**: MIT compatible with Apache 2.0 ✅
- ✅ **Simplicity**: 171-word license vs. 8,600 (Apache) or 5,600 (GPL) ✅
- ✅ **Community Contributions**: Permissive license encourages forks ✅
- ✅ **Bootcamp Use**: Companies can use without restrictions ✅

## References

- MIT License: https://opensource.org/licenses/MIT
- Apache License 2.0: https://www.apache.org/licenses/LICENSE-2.0
- GPL-3.0: https://www.gnu.org/licenses/gpl-3.0.en.html
- Choose a License: https://choosealicense.com/
- ROS 2 License Policy: https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html#licensing

## Revision History

- 2025-12-06: Initial decision - Selected MIT License for all companion code repositories
