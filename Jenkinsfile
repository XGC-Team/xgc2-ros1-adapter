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

rm -rf .work/jenkins debs
.xgc2/scripts/build_debs_in_docker.sh \
  --work-dir "${WORKSPACE}/.work/jenkins" \
  --output-dir "${WORKSPACE}/debs"
'''
      }
    }
  }

  post {
    always {
      archiveArtifacts artifacts: 'debs/*.deb', allowEmptyArchive: true
      sh 'rm -rf .work/jenkins'
    }
  }
}
