#!/usr/bin/env bash
set +x
set -euo pipefail

if [[ -z "${SONAR_TOKEN:-}" ]]; then
    echo "::warning::Skipping Sonar analysis: SONAR_TOKEN is unavailable (expected for fork pull requests)."
    exit 0
fi

workspace="${1:?Expected the industrial_ci target workspace}"
repository="$workspace/src/${2:?Expected the repository name}"
database="$workspace/build/sonar-compile-commands.json"

if [[ ! -f "$repository/sonar-project.properties" ]]; then
    echo "Sonar configuration not found in $repository" >&2
    exit 1
fi

mapfile -d '' databases < <(
    find "$workspace/build" -mindepth 2 -name compile_commands.json -print0
)

if [[ "${#databases[@]}" -eq 0 ]]; then
    echo "No package compilation databases found in $workspace/build" >&2
    exit 1
fi

jq -e -s 'add | if type == "array" and length > 0 then . else error("Empty compilation database") end' \
    "${databases[@]}" > "$database"

scanner_args=("-Dsonar.cfamily.compile-commands=$database")
if [[ "${EVENT_NAME:-}" == "pull_request" ]]; then
    scanner_args+=(
        "-Dsonar.pullrequest.key=${PR_NUMBER:?Missing PR number}"
        "-Dsonar.pullrequest.branch=${PR_BRANCH:?Missing PR branch}"
        "-Dsonar.pullrequest.base=${PR_BASE:?Missing PR base}"
    )
else
    scanner_args+=("-Dsonar.branch.name=${BRANCH:?Missing branch name}")
fi

if command -v sonar-scanner > /dev/null 2>&1; then
    scanner="$(command -v sonar-scanner)"
else
    scanner_version=8.1.0.6389
    scanner_sha256=bb8f709f9cb73352f8d1260a3b3c506c0f41146754bc630762c126d795499d0b
    scanner_directory="$(mktemp -d)"
    trap 'rm -rf "$scanner_directory"' EXIT
    curl --fail --silent --show-error --location --retry 3 \
        --proto '=https' --proto-redir '=https' \
        "https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-${scanner_version}-linux-x64.zip" \
        --output "$scanner_directory/scanner.zip"
    printf '%s  %s\n' "$scanner_sha256" "$scanner_directory/scanner.zip" | sha256sum --check --strict
    unzip -q "$scanner_directory/scanner.zip" -d "$scanner_directory"
    scanner="$scanner_directory/sonar-scanner-${scanner_version}-linux-x64/bin/sonar-scanner"
fi

cd "$repository"
"$scanner" "${scanner_args[@]}"
