// See LICENSE for license details.

package firesim.passes

import firesim.util.{BuildStrategy}
import midas.{DesiredHostFrequency}
import midas.stage.phases.ConfigParametersAnnotation
import midas.stage.GoldenGateOutputFileAnnotation

import freechips.rocketchip.config.Parameters

import firrtl._
import firrtl.passes._
import firrtl.annotations._
import java.io.{File, FileWriter}

/** Generate additional output files for EC2 F1 host platforms, including script
  *  to configure a PLL with the desired frequency and to pick a desired
  *  build strategy
  */
object EC2F1Artefacts extends Transform {
  def inputForm: CircuitForm = LowForm
  def outputForm: CircuitForm = LowForm
  override def name = "[Golden Gate] EC2 F1 Artefact Generation"

  // Emit SH variables to control the FPGA compilation flow
  def shEnvAnno(implicit hostParams: Parameters): GoldenGateOutputFileAnnotation =  {
    val requestedFrequency = hostParams(DesiredHostFrequency)
    val buildStrategy      = hostParams(BuildStrategy)
    val constraints = s"""# FireSim Generated Environment Variables
frequency=${requestedFrequency}
${buildStrategy.emitSh}
"""
    GoldenGateOutputFileAnnotation(constraints, fileSuffix = ".env.sh")
  }

  def execute(state: CircuitState): CircuitState = {
    implicit val p = state.annotations.collectFirst({ case ConfigParametersAnnotation(p)  => p }).get
    state.copy(annotations = shEnvAnno +: state.annotations)
  }
}
