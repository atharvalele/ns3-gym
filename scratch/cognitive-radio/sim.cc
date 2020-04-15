/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Technische Universität Berlin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz@tkn.tu-berlin.de>
 * Modified: Atharva Lele <itsatharva@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/packet-sink.h"
#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/packet-sink.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/stats-module.h"
#include "ns3/flow-monitor-module.h"

#include "mygym.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Cognitive-Radio-RL");

int
main (int argc, char* argv[])
{
  // ns3 parameters
  uint32_t simSeed = 1;
  double simulationTime = 10; // in seconds
  double envStepTime = 0.1; // ns3gym interval for each step
  uint32_t openGymPort = 5555; // default port for connection
  
  // parameters for simulation
  uint32_t numNodes = 2;
  double distance = 10.0; // in metres
  bool fading = false; // disable channel fading
  double puPower = 10; // our "interference" is PU occupying the channel

  // PU access time
  double puAccessTime = 0.1; // in seconds

  // lets consider a repeating pattern, 10 PUs over 10 channels
  std::map<uint32_t, std::vector<uint32_t>> puPattern;
  for (int i = 0; i < 10; i++) {
    std::vector<uint32_t> temp(10, 0);
    temp.at(i) = 1;
    puPattern.insert(std::pair<uint32_t, std::vector<uint32_t>> (i, temp));
  }
  
  // set channel numbers
  uint32_t channNum = puPattern.at(0).size();

  CommandLine cmd;
  // required parameters for OpenGym interface
  cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
  cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", simSeed);
  // optional parameters
  cmd.AddValue ("simTime", "Simulation time in seconds. Default: 10s", simulationTime);
  cmd.AddValue ("enableFading", "If fading should be enabled. Default: false", fading);
  cmd.Parse (argc, argv);

  NS_LOG_UNCOND("Ns3Env parameters:");
  NS_LOG_UNCOND("--simulationTime: " << simulationTime);
  NS_LOG_UNCOND("--openGymPort: " << openGymPort);
  NS_LOG_UNCOND("--envStepTime: " << envStepTime);
  NS_LOG_UNCOND("--seed: " << simSeed);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (simSeed);

  // OpenGym Environment parameters
  Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface> (openGymPort);
  Ptr<MyGymEnv> myGymEnv = CreateObject<MyGymEnv> (channNum);
  myGymEnv->SetOpenGymInterface(openGymInterface);

  NodeContainer nodes;
  nodes.Create (numNodes);

  // Channel Model
  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
  Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel> ();
  Ptr<NakagamiPropagationLossModel> fadingModel = CreateObject<NakagamiPropagationLossModel> ();
  if (fading) {
    lossModel->SetNext (fadingModel);
  }
  spectrumChannel->AddPropagationLossModel (lossModel);
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  spectrumChannel->SetPropagationDelayModel (delayModel);

  // Mobility model
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (distance),
                                 "DeltaY", DoubleValue (distance),
                                 "GridWidth", UintegerValue (numNodes),  // will create linear topology
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  // Channel models, 20MHz BW, side by side no gap
  std::map<uint32_t, Ptr<SpectrumModel> > spectrumModels;
  for (uint32_t chanId=0; chanId<channNum; chanId++) {
    double fc = 5200e6 + 20e6 * chanId;
    BandInfo bandInfo;
    bandInfo.fc = fc;
    bandInfo.fl = fc - 10e6;
    bandInfo.fh = fc + 10e6;
    Bands bands;
    bands.push_back (bandInfo);
    Ptr<SpectrumModel> sm = Create<SpectrumModel> (bands);
    spectrumModels.insert(std::pair<uint32_t, Ptr<SpectrumModel>>(chanId, sm));
  }

  // Spectrum Analyzer for Channel Sensing
  Ptr<Node> sensingNode = nodes.Get (0);
  SpectrumAnalyzerHelper spectrumAnalyzerHelper;
  spectrumAnalyzerHelper.SetChannel (spectrumChannel);
  spectrumAnalyzerHelper.SetPhyAttribute ("Resolution", TimeValue (MilliSeconds (100)));
  spectrumAnalyzerHelper.SetPhyAttribute ("NoisePowerSpectralDensity", DoubleValue (1e-15));     // -120 dBm/Hz
  NetDeviceContainer spectrumAnalyzers;

  for (uint32_t chanId=0; chanId<channNum; chanId++) {
    spectrumAnalyzerHelper.SetRxSpectrumModel (spectrumModels.at(chanId));
    spectrumAnalyzers.Add (spectrumAnalyzerHelper.Install (sensingNode));
  }

  for (uint32_t i=0; i< spectrumAnalyzers.GetN(); i++)
  {
    Ptr<NetDevice> netDev = spectrumAnalyzers.Get(i);
    Ptr<NonCommunicatingNetDevice> nonCommNetDev = DynamicCast<NonCommunicatingNetDevice>(netDev);
    Ptr<Object> spectrumPhy = nonCommNetDev->GetPhy();
    Ptr<SpectrumAnalyzer> spectrumAnalyzer = DynamicCast<SpectrumAnalyzer>(spectrumPhy);
    spectrumAnalyzer->Start ();

    std::ostringstream oss;
    oss.str ("");
    uint32_t devId = netDev->GetIfIndex();
    oss << "/NodeList/" << sensingNode->GetId () << "/DeviceList/" << devId << "/$ns3::NonCommunicatingNetDevice/Phy/AveragePowerSpectralDensityReport";
    uint32_t channelId = i;
    Config::ConnectWithoutContext (oss.str (),MakeBoundCallback (&MyGymEnv::PerformCca, myGymEnv, channelId));
  }

  // Signal Generator --- Generate PU pattern
  Ptr<Node> interferingNode = nodes.Get(1);
  WaveformGeneratorHelper waveformGeneratorHelper;
  waveformGeneratorHelper.SetChannel (spectrumChannel);
  waveformGeneratorHelper.SetPhyAttribute ("Period", TimeValue (Seconds (puAccessTime)));
  waveformGeneratorHelper.SetPhyAttribute ("DutyCycle", DoubleValue (1.0));
  NetDeviceContainer waveformGeneratorDevices;

  for (uint32_t chanId=0; chanId<channNum; chanId++) {
    Ptr<SpectrumValue> wgPsd = Create<SpectrumValue> (spectrumModels.at(chanId));
    *wgPsd = puPower / (20e6);
    NS_LOG_DEBUG ("wgPsd : " << *wgPsd << " integrated power: " << Integral (*(GetPointer (wgPsd))));
    waveformGeneratorHelper.SetTxPowerSpectralDensity (wgPsd);
    waveformGeneratorDevices.Add(waveformGeneratorHelper.Install (interferingNode));
  }

  // Schedule PU pattern
  double scheduledTime = 0;
  while (scheduledTime < simulationTime)
  {
    std::map<uint32_t, std::vector<uint32_t> >::iterator it;
    for (it=puPattern.begin(); it!=puPattern.end(); it++) {
      std::vector<uint32_t> channels = (*it).second;

      for (uint32_t chanId = 0; chanId < channels.size(); chanId++){
        uint32_t occupied = channels.at(chanId);
        NS_LOG_DEBUG("scheduledTime: " << scheduledTime << " ChanId " << chanId << " Occupied: " << occupied);
        if (occupied == 1) {
          Simulator::Schedule (Seconds (scheduledTime), &WaveformGenerator::Start,
                               waveformGeneratorDevices.Get (chanId)->GetObject<NonCommunicatingNetDevice> ()->GetPhy ()->GetObject<WaveformGenerator> ());
        } else {
          Simulator::Schedule (Seconds (scheduledTime), &WaveformGenerator::Stop,
                               waveformGeneratorDevices.Get (chanId)->GetObject<NonCommunicatingNetDevice> ()->GetPhy ()->GetObject<WaveformGenerator> ());
        }
      }
      scheduledTime += puAccessTime;
    }
  }

  NS_LOG_UNCOND ("Simulation start");
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();
  NS_LOG_UNCOND ("Simulation stop");
  myGymEnv->NotifySimulationEnd();

  Simulator::Destroy ();
  NS_LOG_UNCOND ("Simulation exit");
}
