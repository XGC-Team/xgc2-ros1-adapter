pipeline {
  agent any

  options {
    timestamps()
    timeout(time: 45, unit: 'MINUTES')
    disableConcurrentBuilds()
  }

  stages {
    stage('Catkin make in Docker') {
      steps {
        sh '''#!/usr/bin/env bash
set -euo pipefail

work_dir="${WORKSPACE}@tmp/catkin-work"
rm -rf "${work_dir}" debs
.xgc2/scripts/build_debs_in_docker.sh \
  --work-dir "${work_dir}" \
  --output-dir "${WORKSPACE}/debs"
'''
      }
    }
  }

  post {
    always {
      archiveArtifacts artifacts: 'debs/*.deb', allowEmptyArchive: true
      sh 'rm -rf "${WORKSPACE}@tmp/catkin-work"'
    }
  }
}
