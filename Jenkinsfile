pipeline {
  agent any

  options {
    timestamps()
    timeout(time: 45, unit: 'MINUTES')
    disableConcurrentBuilds()
    buildDiscarder(logRotator(numToKeepStr: '20', artifactNumToKeepStr: '0'))
  }

  stages {
    stage('Catkin make in Docker') {
      steps {
        sh '''#!/usr/bin/env bash
set -euo pipefail

rm -rf debs
.xgc2/scripts/build_debs_in_docker.sh --skip-output-copy
'''
      }
    }
  }

  post {
    always {
      sh 'rm -rf debs'
      deleteDir()
    }
  }
}
