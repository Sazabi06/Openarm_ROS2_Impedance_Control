---
name: karpathy-guidelines
description: Core coding guidelines inspired by Andrej Karpathy's observations on LLM coding pitfalls, focusing on thinking before coding, simplicity, surgical changes, and goal-driven execution.
version: 1.0.0
---
# Andrej Karpathy's Coding Guidelines

These guidelines represent core principles inspired by Andrej Karpathy's observations on common LLM coding pitfalls and failure modes. Adhere to these principles rigorously during web application development, debugging, and systems engineering.

## 1. Think Before Coding
- **State Assumptions**: Before executing any code changes, explicitly state your assumptions and outline the trade-offs of the proposed solution.
- **Clarify Ambiguity**: If a request is ambiguous or there are multiple potential implementation paths, propose the options and ask for user clarification rather than guessing.
- **Identify Uncertainty**: If you are uncertain about a system component, dependency, or side effect, stop and call it out immediately.

## 2. Simplicity First
- **Minimal Code**: Write the minimum amount of code necessary to fully solve the problem.
- **No Speculative Abstractions**: Avoid speculative abstractions, unnecessary flexibility, or features that were not explicitly requested.
- **Prioritize the Simplest Approach**: If a simpler, more direct implementation exists, default to it.

## 3. Surgical Changes
- **Targeted Edits**: Touch only the files, lines, and components directly required by the task.
- **No Unrelated Refactoring**: Do not reform, "improve", or refactor adjacent or unrelated code or comments.
- **Report, Don't Delete**: If you notice unrelated issues (e.g. dead code, suboptimal patterns), document them in a comment or report them rather than making silent changes.

## 4. Goal-Driven Execution
- **Verifiable Targets**: Propose clear, measurable criteria for success before implementing the solution.
- **Test First / Early**: Define the verification plan (e.g. unit tests, command-line runs, or browser testing steps) upfront.
- **Verify Before Completion**: Ensure all verification criteria are successfully met and documented before concluding the task.
