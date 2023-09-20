pipeline {
  agent any
  stages {
    stage('build') {
      steps {
        cmakeBuild(installation: 'InSearchPath', buildType: 'Release', generator: 'Visual Studio 16 2019', label: 'Build with VS 2019', sourceDir: '/')
      }
    }

  }
}