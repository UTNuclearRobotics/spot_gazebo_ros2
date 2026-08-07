/*
 * BarcodeScanner — Zebra Symbol LS2208 1-D laser scanner system plugin for
 * Gazebo Ignition Fortress (ign-gazebo 6.x).
 *
 * WHY THIS IS NOT SHAPED LIKE RfidReader
 * --------------------------------------
 * RfidReader models an RF link: a broad lobe, graded path loss, many tags per
 * interrogation cycle, probabilistic reads that improve with dwell and power.
 * The LS2208 is a 650 nm Class 2 laser diode sweeping a single line at 100
 * scans/sec over a 35 deg scan angle. Consequences that drive every design
 * choice below:
 *
 *   1. ONE decode per trigger pull. The scanner is an input device: it emits a
 *      single string and stops. It does NOT enumerate everything in view. An
 *      inventory survey that swept a rack face with RFID must become a
 *      per-asset aim-and-trigger sequence.
 *   2. Range is a DEPTH OF FIELD, not a falloff. Outside [dof_near, dof_far]
 *      the symbol simply does not resolve. Both limits are hard, and BOTH are
 *      a function of printed label density (mil), not of the scanner alone.
 *   3. Light does not pass through the rack. Occlusion is binary, unlike the
 *      dB attenuation RfidReader charges.
 *   4. Orientation is gated on three independent axes with different limits,
 *      and for a short-bar label ROLL IS SET BY LABEL ASPECT RATIO, not by the
 *      datasheet. See <auto_roll_limit>.
 *
 * DATASHEET — Zebra "LS2208 Handheld Scanner Product Spec Sheet", part no.
 * SS-LS2208, 10/2015. THIS IS THE AUTHORITATIVE SOURCE for every value below.
 *
 *   An earlier draft of this plugin was sourced from the Motorola-branded
 *   sheet (SS-LS2208, 12/08, (c) 2007) shipped with LS2208-SR20007R-UR, and
 *   asserted that the two revisions were "identical in every Performance
 *   Characteristic". THAT WAS WRONG. The revisions disagree on the orientation
 *   tolerances and on the entire depth-of-field table:
 *
 *                       12/08 (superseded)      10/2015 (current)
 *     Roll (tilt)       +/- 30 deg              +/- 60 deg
 *     Skew (yaw)        +/- 60 deg              +/- 10 deg
 *     Pitch             +/- 65 deg              +/- 65 deg   (unchanged)
 *     Scan angle        not stated              35 deg       (now stated)
 *     DOF rows          5 / 7.5 / 10 / 20 / 40  3 / 4 / 5 / 7.5 / 20 / 40 mil
 *                       mil + 13 mil UPC        + 13 mil UPC
 *
 *   Roll and skew are very nearly transposed between the two, so a file
 *   carrying 30/60 is not merely stale, it gates the WRONG AXIS. Skew
 *   tightening 60 -> 10 deg is the single largest behavioural change: with the
 *   exponent-4 skew term in the margin, off-boresight approaches that used to
 *   decode reliably now fail outright.
 *
 *   Scanner type        Bi-directional
 *   Light source        650 nm Laser Diode Class 2
 *   Scan pattern        Single line
 *   Scan rate           100 scans/sec
 *   Scan angle          35 deg
 *   Print contrast      20% minimum reflective difference
 *   Skew tolerance      +/- 10 deg
 *   Pitch tolerance     +/- 65 deg
 *   Roll tolerance      +/- 60 deg
 *   Min element res.    Code 39: 3.0 mil   (stated, but not modelled: the
 *                       plugin has no concept of module width beyond the
 *                       <beam_spot> vs narrow-module check you make by hand)
 *   Decode capability   includes Code 128 and Code 128 Full ASCII
 *   Depth of field      Code 39  3   mil   1.6" -  4.8"   0.0406 - 0.1219 m
 *                       Code 39  4   mil   1.3" -  8.7"   0.0330 - 0.2210 m
 *                       Code 39  5   mil   0.9" - 11.7"   0.0229 - 0.2972 m
 *                       Code 39  7.5 mil   0.1" - 19.1"   0.0025 - 0.4851 m
 *                       100% UPC 13  mil   0    - 25.1"   0      - 0.6375 m
 *                       Code 39  20  mil   0    - 33.8"   0      - 0.8585 m
 *                       Code 39  40  mil   1"   - 43.1"   0.0254 - 1.0947 m
 *
 *   Metric values above are converted from the inch column, which is
 *   authoritative. Unlike the 12/08 sheet — whose metric column carried two
 *   transcription errors (10 mil far "14.25 cm" for 36.2 cm; 5 mil near "6 cm"
 *   for 6.35 cm) — the 10/2015 metric column agrees with the inch column to
 *   the rounding in every row. The 10 mil row is simply gone; do not
 *   interpolate one, pick the printed density you actually use.
 *
 *   The 40 mil near limit really is 1", larger than the 20 mil row's 0". That
 *   is what the sheet says; it is not a typo we are correcting.
 *
 *   The table gives no Code 128 row. Depth of field is governed by the narrow
 *   element width, not the symbology, so a Code 39 row may be read across to a
 *   Code 128 symbol of the same X dimension — but it IS a substitution, and it
 *   is the reason <dof_near>/<dof_far> are exposed rather than hard-coded.
 *
 *   WE DO NOT SHIP ANY ROW OF THIS TABLE. Our printed symbol measures 28.0 mm
 *   first bar to last bar including quiet zones, over 132 modules, giving an X
 *   dimension of 28.0/132 = 0.2121 mm = 8.35 mil. There is no 8.35 mil row.
 *   The defaults are therefore DERIVED, and both differ from the sheet:
 *
 *     <dof_far>  0.50873 m. Linear interpolation between the 7.5 mil row
 *                (19.1") and the 13 mil UPC row (25.1"):
 *                  19.1 + (25.1-19.1)*(8.3512-7.5)/(13-7.5) = 20.03"
 *                Crossing a Code 39 row with a UPC row is defensible only
 *                because DOF tracks the narrow element width, not the
 *                symbology — the same argument that licenses the Code 128
 *                substitution above. It is still an interpolation, not a spec.
 *
 *     <dof_near> 0.04440 m, and NOT the sheet's 0.1" (0.00254 m). The sheet's
 *                near limits are measured with a short test symbol. A laser
 *                scanner must sweep the ENTIRE symbol, quiet zones included,
 *                within one pass, and the 35 deg scan angle spans only
 *                  2 * d * tan(17.5 deg) = 0.6306 * d
 *                so a 28 mm code needs 0.028/0.6306 = 0.0444 m of standoff
 *                before both ends are in the line at once. Closer than that
 *                the hardware cannot read it at any orientation.
 *
 *                THE PLUGIN DOES NOT MODEL THIS DIRECTLY. The horizontal gate
 *                tests the label ORIGIN against <scan_halfangle>; it never
 *                checks that the symbol's projected width fits inside the
 *                sweep. <dof_near> is standing in for that missing gate, which
 *                is why it is a geometry-derived number and not a datasheet
 *                one. Two consequences: (a) recompute it whenever
 *                <symbol_width> or <scan_halfangle> changes, and (b) it is
 *                only correct on boresight — a wide symbol sitting near the
 *                edge of the sweep needs more standoff than this, and the
 *                plugin will not catch that. If that case starts to matter,
 *                add a real width gate rather than inflating <dof_near>.
 *
 * NOT IN THE DATASHEET — do not cite these as spec values
 *   <symbol_width>, <symbol_height>
 *                       Properties of OUR printed label, not the scanner.
 *                       They set the auto roll limit; see <auto_roll_limit>.
 *   <beam_spot>         Not specified. 0.2 mm is derived from the requirement
 *                       that the spot resolve one narrow module. At the
 *                       measured 8.3512 mil that module is 0.21212 mm, so
 *                       0.2 mm fits with about 5.7% to spare.
 *
 *                       NOTE this rule was VIOLATED under the old 7.5 mil
 *                       assumption, where the module was 0.1905 mm and the
 *                       0.2 mm spot was 5% WIDER than the element it was
 *                       supposed to resolve. Nobody noticed because the
 *                       plugin never enforces the relation — it is a
 *                       hand-checked invariant, not an assertion. Re-check it
 *                       by hand whenever <symbol_width> changes.
 *
 *                       It widens the vertical acceptance band and the roll
 *                       limit, so it is not cosmetic.
 *   <rescan_delay>      Not a spec-sheet value. The re-read timeout is a
 *                       host-programmable parameter ("Timeout Between Decodes,
 *                       Same Symbol") documented in the Product Reference
 *                       Guide, not here. 0.5 s is a placeholder.
 *   <specular_deadzone>, <print_contrast> vs <label_contrast>,
 *   <p_decode_max>, <margin_noise>
 *                       Sim-side modelling knobs. Only the 20% minimum
 *                       reflective difference in <print_contrast> is a
 *                       datasheet figure; what the simulated label stock
 *                       achieves is our choice.
 *
 *   Ambient light immunity is specified — the 10/2015 sheet quantifies it as
 *   "0 to 8,000 Foot Candles / 0 to 86,080 Lux", where the 12/08 sheet gave
 *   only prose — and is NOT modelled at all either way. World lighting has no
 *   effect on decoding here.
 *
 * SDF configuration (attach as a MODEL plugin to the robot):
 *
 *   <plugin filename="BarcodeScanner" name="barcode::BarcodeScanner">
 *     <scanner_id>ls2208</scanner_id>
 *     <label_prefix>barcode_label_</label_prefix>
 *
 *     <!-- The payload IS the model name suffix. A model named
 *          barcode_label_031FB0D decodes to the string "031FB0D". This
 *          mirrors the hardware: the scanner knows only the code, and the
 *          inventory system resolves it against rack_tag_manifest.csv. It also
 *          removes the ground-truth leak that semantic tag names
 *          (rfid_tag_r1_ups) handed the estimator for free. -->
 *
 *     <update_rate>100</update_rate>   scans/sec; datasheet typical
 *     <mode>trigger</mode>             trigger | presentation
 *                                      trigger      = handheld, one decode per
 *                                                     pull, latched until
 *                                                     release
 *                                      presentation = Intellistand hands-free,
 *                                                     continuous, with
 *                                                     <rescan_delay> before the
 *                                                     same code repeats
 *     <rescan_delay>0.5</rescan_delay>            [s] presentation mode only.
 *                                      Quiet period after ANY decode, not just
 *                                      a repeat of the same code — the
 *                                      hardware's re-read timer is global, so
 *                                      a different label swept during the
 *                                      window is also refused.
 *     <topic>/barcode/scans</topic>
 *     <trigger_topic>/barcode/trigger</trigger_topic>
 *     <publish_tf>true</publish_tf>
 *     <tf_topic>/barcode/tf</tf_topic>
 *     <seed>42</seed>                  always logged; omit = random
 *
 *     <!-- Exit window pose. Boresight is +X, the scan line sweeps in the
 *          scanner XY plane, bar height is measured along scanner Z. -->
 *     <link>arm0_hand</link>
 *     <pose>0.10473 0.0049 -0.1117 0 0 0</pose>
 *     <parent_frame>arm0_hand</parent_frame>      ROS parent for TF
 *
 *     <!-- Depth of field for the density you actually printed. A 7-character
 *          Code 128 Subset B payload is
 *            11*7 data + 11 start + 11 check + 13 stop  = 112 modules
 *            + 10 modules quiet zone each side          = 132 modules
 *          and the measured symbol is 28.0 mm across, so
 *            X = 28.0 / 132 = 0.2121 mm = 8.35 mil
 *          Neither default below is a datasheet cell — dof_far interpolates
 *          the 7.5 and 13 mil rows, dof_near is the scan-line-width floor.
 *          See the DEPTH OF FIELD discussion in the header block. -->
 *     <dof_near>0.04440</dof_near>
 *     <dof_far>0.50873</dof_far>
 *
 *     <!-- Printed SYMBOL extent, including quiet zones. Not the substrate,
 *          and not the generator's PRINT_W, which is only a fit ceiling.
 *          Measure this: it drives dof_near AND the auto roll limit. -->
 *     <symbol_width>0.028</symbol_width>
 *     <symbol_height>0.003</symbol_height>
 *
 *     <scan_halfangle>17.5</scan_halfangle>   [deg] half of the datasheet's
 *                                             35 deg scan angle
 *     <beam_spot>0.0002</beam_spot>           [m] spot diameter; must be <=
 *                                             one narrow module to resolve.
 *                                             0.2 mm against a 0.21212 mm
 *                                             module: 5.7% of headroom. The
 *                                             old 7.5 mil module was 0.1905
 *                                             mm, i.e. NARROWER than the spot
 *                                             — that configuration broke this
 *                                             rule. Do not raise it.
 *
 *     <auto_roll_limit>true</auto_roll_limit>
 *          Derive the roll limit from label aspect instead of the datasheet.
 *          A line rolled by theta drifts symbol_width*tan(theta) vertically
 *          across the symbol and must stay inside the bar height, widened by
 *          the beam spot:
 *              atan((symbol_height + beam_spot) / symbol_width)
 *              atan((3.0 + 0.2) / 28.0) = 6.52 deg
 *          NOT the +/-60 deg the spec sheet quotes, which assumes a
 *          full-height label. Set false to use <roll_limit> verbatim.
 *
 *          The denominator is the SYMBOL width including quiet zones. On the
 *          measured 28 mm label that now coincides with the generator's
 *          PRINT_W, so the old warning about not confusing the two no longer
 *          bites — but the two remain different quantities and can diverge
 *          again the moment the payload length or the mil changes. The beam
 *          spot in the numerator is what separates 6.52 deg from a bare
 *          atan(3/28) = 6.12 deg; do not drop it.
 *
 *          Because the geometric 6.52 deg is far below either revision's
 *          datasheet roll figure, the min() means <roll_limit> is INERT while
 *          auto_roll_limit is true and the label stays short-bar. Correcting
 *          30 -> 60 therefore changes no decode outcome today; it matters only
 *          if you set auto_roll_limit false or print a full-height label.
 *
 *          Note the 25.146 mm symbol this file used to assume gave 7.25 deg.
 *          The wider real label is a TIGHTER roll gate, not a looser one:
 *          a longer symbol means more vertical drift per degree of roll.
 *     <roll_limit>60</roll_limit>      [deg] datasheet ceiling; also clamps
 *                                      the auto value
 *     <pitch_limit>65</pitch_limit>    [deg]
 *     <skew_limit>10</skew_limit>      [deg] NOT 60 — see the revision table
 *                                      above. This one really does bite.
 *
 *     <specular_deadzone>0</specular_deadzone>
 *          [deg] refuse decodes closer than this to normal incidence. Zero for
 *          matte thermal-transfer labels. Raise to ~5 to model the glossy
 *          laminate that makes an operator tilt the gun off perpendicular.
 *
 *     <print_contrast>0.20</print_contrast>    minimum reflective difference
 *     <label_contrast>0.85</label_contrast>    what the sim's labels achieve
 *
 *     <p_decode_max>0.99</p_decode_max>        per-scan ceiling
 *     <margin_noise>0.05</margin_noise>        per-scan jitter on the margin
 *
 *     <occluder>                       repeat per opaque volume. Unlike the
 *                                      RfidReader element of the same name
 *                                      there is no dB: any chord > 0 blocks
 *                                      the beam completely.
 *       <name>rack_body</name>
 *       <model>server_rack</model>     optional: ride a model's pose
 *       <pose>0 0.035 1.015 0 0 0</pose>
 *       <size>0.60 1.03 2.03</size>
 *     </occluder>
 *   </plugin>
 *
 * Trigger commands on <trigger_topic>, JSON StringMsg:
 *   {"trigger":true}                 pull   (trigger mode)
 *   {"trigger":false}                release
 *   {"mode":"presentation"}          switch to hands-free
 *   {"mode":"trigger"}               switch back
 *
 * JSON scan report on <topic>:
 *   {"t":12.400,"scan":124,"scanner":"ls2208","mode":"trigger",
 *    "trigger":true,
 *    "decoded":[{"data":"031FB0D","symbology":"CODE128","r":0.152,
 *                "roll":2.1,"pitch":-8.4,"skew":12.0,"margin":0.63}],
 *    "n_labels":138}
 *
 * "decoded" holds AT MOST ONE entry, always. If several labels are in view and
 * in tolerance the best-margin candidate wins, exactly as the hardware picks
 * whichever symbol the line happens to fully cross first.
 *
 * A report is published every scan the trigger is active AND unlatched,
 * including when "decoded" is empty. An empty report is evidence — this pose,
 * nothing resolved — and is the input to non-detection scoring. Do not drop,
 * throttle or coalesce it downstream.
 *
 * Reports STOP in two cases, and downstream cannot tell them apart from the
 * report topic alone:
 *   - the trigger is released; a released LS2208 emits nothing at all
 *   - the trigger is still held but a decode already succeeded, so the
 *     single-decode latch has closed for this pull
 * Either way, silence means "not scanning" and an empty "decoded" list means
 * "scanning and resolving nothing". Conflating the two will make a successful
 * decode look like a long run of non-detections. The scanner's TF frame IS
 * broadcast unconditionally, so tf2 never goes stale between pulls.
 *
 * "r", "roll", "pitch", "skew", "margin" and "n_labels" are ground truth with
 * no hardware counterpart. Validation only. The hardware gives you the string
 * and nothing else.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>

namespace barcode
{

class BarcodeScanner
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPostUpdate
{
  /// \brief A printed label. The barcode plane normal is the label's local
  /// -Y, the symbol's long axis is local +X and bar height is local +Z. This
  /// matches the rack tag convention: tags sit at y = -0.503 on a faceplate
  /// whose outward normal is -Y, so an unrotated label already faces the
  /// aisle correctly.
  private: struct LabelInfo
  {
    std::string name;       ///< full model name
    std::string payload;    ///< model name minus <label_prefix>
  };

  /// \brief An opaque volume. Binary: any intersection blocks the beam.
  private: struct Occluder
  {
    std::string name;
    std::string modelName;      ///< optional: pose is relative to this model
    ignition::math::Pose3d pose;
    ignition::math::Vector3d size;
    ignition::gazebo::Entity model{ignition::gazebo::kNullEntity};
    bool warned{false};
  };

  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager & /*_eventMgr*/) override
  {
    this->model = ignition::gazebo::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      ignerr << "[BarcodeScanner] Plugin must be attached to a model.\n";
      return;
    }

    this->scannerId =
        _sdf->Get<std::string>("scanner_id", this->scannerId).first;
    this->labelPrefix =
        _sdf->Get<std::string>("label_prefix", this->labelPrefix).first;
    this->updateRate =
        _sdf->Get<double>("update_rate", this->updateRate).first;
    if (this->updateRate <= 0.0)
    {
      ignwarn << "[BarcodeScanner] <update_rate> must be > 0; using 100.\n";
      this->updateRate = 100.0;
    }
    this->period = std::chrono::duration<double>(1.0 / this->updateRate);

    const std::string modeStr =
        _sdf->Get<std::string>("mode", std::string("trigger")).first;
    if (modeStr == "presentation")
    {
      this->presentation = true;
      this->triggerHeld = true;   // Intellistand is always live
    }
    else if (modeStr != "trigger")
    {
      ignwarn << "[BarcodeScanner] <mode> '" << modeStr
              << "' unrecognized; expected 'trigger' or 'presentation'. "
              << "Using 'trigger'.\n";
    }
    this->rescanDelay =
        _sdf->Get<double>("rescan_delay", this->rescanDelay).first;

    this->publishTf = _sdf->Get<bool>("publish_tf", this->publishTf).first;

    // ---- optics / geometry ----------------------------------------------
    this->dofNear = _sdf->Get<double>("dof_near", this->dofNear).first;
    this->dofFar  = _sdf->Get<double>("dof_far", this->dofFar).first;
    if (this->dofNear > this->dofFar)
    {
      ignerr << "[BarcodeScanner] <dof_near> (" << this->dofNear
             << ") > <dof_far> (" << this->dofFar << "); swapping.\n";
      std::swap(this->dofNear, this->dofFar);
    }

    this->symbolWidth =
        _sdf->Get<double>("symbol_width", this->symbolWidth).first;
    this->symbolHeight =
        _sdf->Get<double>("symbol_height", this->symbolHeight).first;
    this->beamSpot = _sdf->Get<double>("beam_spot", this->beamSpot).first;

    const double scanHalf =
        _sdf->Get<double>("scan_halfangle", 17.5).first;
    this->scanHalfAngle = scanHalf * M_PI / 180.0;

    const double rollLimDeg = _sdf->Get<double>("roll_limit", 60.0).first;
    this->rollLimit = rollLimDeg * M_PI / 180.0;
    this->autoRollLimit =
        _sdf->Get<bool>("auto_roll_limit", this->autoRollLimit).first;
    if (this->autoRollLimit)
    {
      // A line rolled by theta drifts symbolWidth*tan(theta) vertically over
      // the full symbol. It must land inside the bar height, widened by the
      // beam spot. Short bars dominate the datasheet number by a wide margin.
      if (this->symbolWidth > 1e-9)
      {
        const double geo = std::atan2(
            this->symbolHeight + this->beamSpot, this->symbolWidth);
        this->rollLimit = std::min(this->rollLimit, geo);
      }
      ignmsg << "[BarcodeScanner] auto_roll_limit: symbol "
             << this->symbolWidth * 1e3 << " x " << this->symbolHeight * 1e3
             << " mm gives a roll limit of "
             << this->rollLimit * 180.0 / M_PI << " deg (datasheet ceiling "
             << rollLimDeg << " deg).\n";
    }

    this->pitchLimit =
        _sdf->Get<double>("pitch_limit", 65.0).first * M_PI / 180.0;
    this->skewLimit =
        _sdf->Get<double>("skew_limit", 10.0).first * M_PI / 180.0;
    this->specularDeadzone =
        _sdf->Get<double>("specular_deadzone", 0.0).first * M_PI / 180.0;

    this->printContrast =
        _sdf->Get<double>("print_contrast", this->printContrast).first;
    this->labelContrast =
        _sdf->Get<double>("label_contrast", this->labelContrast).first;
    if (this->labelContrast < this->printContrast)
    {
      ignwarn << "[BarcodeScanner] <label_contrast> ("
              << this->labelContrast << ") is below the LS2208 minimum "
              << "reflective difference <print_contrast> ("
              << this->printContrast << "). NOTHING will ever decode.\n";
    }

    this->pDecodeMax =
        _sdf->Get<double>("p_decode_max", this->pDecodeMax).first;
    this->marginNoise =
        _sdf->Get<double>("margin_noise", this->marginNoise).first;

    // ---- exit window mount ----------------------------------------------
    this->linkName = _sdf->Get<std::string>("link", this->linkName).first;
    this->mountPose =
        _sdf->Get<ignition::math::Pose3d>("pose",
            ignition::math::Pose3d::Zero).first;
    this->parentFrame =
        _sdf->Get<std::string>("parent_frame", this->linkName).first;

    // ---- occluders --------------------------------------------------------
    // Clone: GetElement/GetNextElement are non-const in libsdformat.
    auto sdfClone = _sdf->Clone();
    if (sdfClone->HasElement("occluder"))
    {
      int idx = 0;
      for (auto e = sdfClone->GetElement("occluder"); e;
           e = e->GetNextElement("occluder"), ++idx)
      {
        Occluder o;
        o.name = e->Get<std::string>(
            "name", "occluder" + std::to_string(idx)).first;
        o.modelName = e->Get<std::string>("model", std::string()).first;
        o.pose = e->Get<ignition::math::Pose3d>("pose",
            ignition::math::Pose3d::Zero).first;
        // Default Zero, not One: a missing or misspelt <size> must fail
        // visibly. Defaulting to a unit cube would drop an invisible 1 m
        // blocker into a depth of field only 0.46 m deep and kill every decode
        // in the room with no diagnostic at all.
        o.size = e->Get<ignition::math::Vector3d>("size",
            ignition::math::Vector3d::Zero).first;
        if (o.size.X() <= 0.0 || o.size.Y() <= 0.0 || o.size.Z() <= 0.0)
        {
          ignerr << "[BarcodeScanner] Occluder '" << o.name
                 << "' has a non-positive <size> (" << o.size
                 << "); skipping it. It will block nothing.\n";
          continue;
        }
        this->occluders.push_back(o);
      }
    }

    // ---- rng --------------------------------------------------------------
    if (_sdf->HasElement("seed"))
    {
      this->seedUsed = _sdf->Get<unsigned int>("seed", 0u).first;
    }
    else
    {
      this->seedUsed = static_cast<unsigned int>(
          std::chrono::high_resolution_clock::now()
              .time_since_epoch().count());
    }
    this->rng.seed(this->seedUsed);
    ignmsg << "[BarcodeScanner] seed=" << this->seedUsed << "\n";

    // ---- transport --------------------------------------------------------
    const std::string topic =
        _sdf->Get<std::string>("topic", std::string("/barcode/scans")).first;
    this->pub = this->node.Advertise<ignition::msgs::StringMsg>(topic);

    if (this->publishTf)
    {
      const std::string tfTopic =
          _sdf->Get<std::string>("tf_topic",
              std::string("/barcode/tf")).first;
      this->tfPub = this->node.Advertise<ignition::msgs::Pose_V>(tfTopic);
    }

    // Log the startup banner BEFORE subscribing: once OnTriggerCmd is live it
    // can flip presentation/triggerHeld from the transport thread, and this
    // read is not under the mutex.
    ignmsg << "[BarcodeScanner] '" << this->scannerId << "' on link '"
           << this->linkName << "', mode="
           << (this->presentation ? "presentation" : "trigger")
           << ", DOF " << this->dofNear << ".." << this->dofFar << " m, "
           << "roll/pitch/skew limits "
           << this->rollLimit * 180.0 / M_PI << "/"
           << this->pitchLimit * 180.0 / M_PI << "/"
           << this->skewLimit * 180.0 / M_PI << " deg.\n";

    const std::string trigTopic =
        _sdf->Get<std::string>("trigger_topic",
            std::string("/barcode/trigger")).first;
    this->node.Subscribe(trigTopic, &BarcodeScanner::OnTriggerCmd, this);
  }

  /// \brief Trigger / mode commands. Called on the transport thread, NOT the
  /// PostUpdate thread — hence triggerMutex.
  private: void OnTriggerCmd(const ignition::msgs::StringMsg &_msg)
  {
    const std::string &s = _msg.data();
    std::lock_guard<std::mutex> lock(this->triggerMutex);

    const auto modePos = s.find("\"mode\"");
    if (modePos != std::string::npos)
    {
      if (s.find("presentation", modePos) != std::string::npos)
      {
        this->presentation = true;
        this->triggerHeld = true;
      }
      else if (s.find("trigger", modePos) != std::string::npos)
      {
        this->presentation = false;
        this->triggerHeld = false;
        this->latched = false;
      }
    }

    const auto trigPos = s.find("\"trigger\"");
    if (trigPos != std::string::npos && !this->presentation)
    {
      const auto truePos = s.find("true", trigPos);
      const auto falsePos = s.find("false", trigPos);
      const bool wantHeld =
          (truePos != std::string::npos &&
           (falsePos == std::string::npos || truePos < falsePos));
      if (wantHeld && !this->triggerHeld)
      {
        // Rising edge: a fresh pull re-arms the single-decode latch.
        this->latched = false;
      }
      this->triggerHeld = wantHeld;
      if (!wantHeld)
        this->latched = false;
    }
  }

  /// \brief Chord length [m] of segment _a->_b through an oriented box.
  /// Slab method in the box frame; 0 if the segment misses.
  private: static double SegmentBoxChord(
      const ignition::math::Vector3d &_a,
      const ignition::math::Vector3d &_b,
      const ignition::math::Pose3d &_boxPose,
      const ignition::math::Vector3d &_size)
  {
    const ignition::math::Vector3d a =
        _boxPose.Rot().RotateVectorReverse(_a - _boxPose.Pos());
    const ignition::math::Vector3d b =
        _boxPose.Rot().RotateVectorReverse(_b - _boxPose.Pos());
    const ignition::math::Vector3d d = b - a;
    const ignition::math::Vector3d h = _size * 0.5;

    double t0 = 0.0, t1 = 1.0;
    for (int i = 0; i < 3; ++i)
    {
      const double di = (i == 0) ? d.X() : (i == 1) ? d.Y() : d.Z();
      const double ai = (i == 0) ? a.X() : (i == 1) ? a.Y() : a.Z();
      const double hi = (i == 0) ? h.X() : (i == 1) ? h.Y() : h.Z();
      if (std::abs(di) < 1e-12)
      {
        if (ai < -hi || ai > hi)
          return 0.0;          // parallel to this slab and outside it
        continue;
      }
      double tn = (-hi - ai) / di;
      double tf = ( hi - ai) / di;
      if (tn > tf)
        std::swap(tn, tf);
      t0 = std::max(t0, tn);
      t1 = std::min(t1, tf);
      if (t0 > t1)
        return 0.0;
    }
    return (t1 - t0) * d.Length();
  }

  /// \brief True if any opaque volume intersects the exit-window -> label
  /// segment. Binary, unlike RfidReader::PathAttenuationDb: 650 nm does not
  /// pass through a rack side panel.
  private: bool Occluded(
      const ignition::math::Vector3d &_a,
      const ignition::math::Vector3d &_b,
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    for (auto &o : this->occluders)
    {
      ignition::math::Pose3d pose = o.pose;
      if (!o.modelName.empty())
      {
        if (o.model == ignition::gazebo::kNullEntity)
        {
          o.model = _ecm.EntityByComponents(
              ignition::gazebo::components::Name(o.modelName),
              ignition::gazebo::components::Model());
          if (o.model == ignition::gazebo::kNullEntity)
          {
            if (!o.warned)
            {
              ignwarn << "[BarcodeScanner] Occluder '" << o.name
                      << "': model '" << o.modelName << "' not found (will "
                      << "keep trying); this volume blocks nothing "
                      << "meanwhile.\n";
              o.warned = true;
            }
            continue;
          }
        }
        pose = ignition::gazebo::worldPose(o.model, _ecm) * o.pose;
      }

      if (SegmentBoxChord(_a, _b, pose, o.size) > 0.0)
        return true;
    }
    return false;
  }

  /// \brief Maintain the set of label entities (world models named
  /// <label_prefix>*) via EachNew/EachRemoved. Poses are NOT cached; they are
  /// resolved from the ECM every scan.
  private: void SyncLabels(
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    auto add = [&](const ignition::gazebo::Entity &_e,
                   const std::string &_name)
    {
      LabelInfo li;
      li.name = _name;
      li.payload = _name.substr(this->labelPrefix.size());
      if (li.payload.empty())
      {
        ignwarn << "[BarcodeScanner] Model '" << _name << "' matches "
                << "<label_prefix> but carries no payload suffix; it will "
                << "decode to an empty string.\n";
      }
      this->labels[_e] = li;
    };

    _ecm.EachNew<ignition::gazebo::components::Model,
                 ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &_e,
            const ignition::gazebo::components::Model *,
            const ignition::gazebo::components::Name *_name) -> bool
        {
          if (_name->Data().rfind(this->labelPrefix, 0) == 0)
          {
            add(_e, _name->Data());
            if (this->labelsSeeded)
            {
              ignmsg << "[BarcodeScanner] Label '" << _name->Data()
                     << "' appeared.\n";
            }
          }
          return true;
        });

    _ecm.EachRemoved<ignition::gazebo::components::Model,
                     ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &_e,
            const ignition::gazebo::components::Model *,
            const ignition::gazebo::components::Name *_name) -> bool
        {
          if (this->labels.erase(_e) > 0)
          {
            ignmsg << "[BarcodeScanner] Label '" << _name->Data()
                   << "' removed.\n";
          }
          return true;
        });

    if (!this->labelsSeeded)
    {
      // EachNew only reports entities created this iteration, so labels that
      // existed at world load must be swept once.
      _ecm.Each<ignition::gazebo::components::Model,
                ignition::gazebo::components::Name>(
          [&](const ignition::gazebo::Entity &_e,
              const ignition::gazebo::components::Model *,
              const ignition::gazebo::components::Name *_name) -> bool
          {
            if (_name->Data().rfind(this->labelPrefix, 0) == 0 &&
                this->labels.find(_e) == this->labels.end())
            {
              add(_e, _name->Data());
            }
            return true;
          });
      this->labelsSeeded = true;
      ignmsg << "[BarcodeScanner] Found " << this->labels.size()
             << " labels with prefix '" << this->labelPrefix << "'.\n";
    }
  }

  public: void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (!this->model.Valid(_ecm))
      return;

    // Every iteration, not on the scan cadence: EachNew/EachRemoved report
    // only THIS iteration's changes, so throttling here would miss spawns.
    this->SyncLabels(_ecm);

    if (_info.paused)
      return;

    const std::chrono::duration<double> simTime(_info.simTime);
    if (simTime - this->lastUpdate < this->period)
      return;
    this->lastUpdate = simTime;

    // latched is read-modify-written on BOTH this thread and the transport
    // thread (OnTriggerCmd re-arms it on a rising edge), so it lives under the
    // same mutex as triggerHeld. Without the lock the re-arm can be lost to
    // this thread's latch write in the same instant and the scanner wedges
    // until the next release.
    bool held, pres, isLatched;
    {
      std::lock_guard<std::mutex> lock(this->triggerMutex);
      held = this->triggerHeld;
      pres = this->presentation;
      // Presentation mode re-arms on a timer; trigger mode only on release.
      if (this->latched && pres &&
          simTime.count() - this->lastDecodeTime >= this->rescanDelay)
      {
        this->latched = false;
      }
      isLatched = this->latched;
    }

    // The scanner's own frame is broadcast every cycle whether or not the
    // trigger is held, so tf2 never sees it go stale between pulls — the same
    // reason RfidReader broadcasts all antenna frames unconditionally. It is
    // built here, BEFORE the <link> lookup, because it needs only the mount
    // pose: a mistyped <link> should not also take the TF tree down with it.
    ignition::msgs::Pose_V tfMsg;
    if (this->publishTf)
    {
      this->AddTransform(tfMsg, this->parentFrame,
          "barcode_scanner_" + this->scannerId, this->mountPose, simTime);
    }

    // ---- resolve the exit window ------------------------------------------
    if (this->link == ignition::gazebo::kNullEntity)
    {
      // A bad <link> does NOT fail loudly: warn once, then never decode.
      // Reports keep publishing empty, indistinguishable from an empty rack.
      // Check this warning before trusting a run of no reads.
      this->link = this->model.LinkByName(_ecm, this->linkName);
      if (this->link == ignition::gazebo::kNullEntity)
      {
        if (!this->linkWarned)
        {
          // URDF fixed-joint children (virtual hand/tool0 frames) are lumped
          // into their parent link by URDF->SDF and won't appear here.
          std::string available;
          for (const auto &l : this->model.Links(_ecm))
          {
            auto *n = _ecm.Component<ignition::gazebo::components::Name>(l);
            if (n)
              available += " '" + n->Data() + "'";
          }
          ignwarn << "[BarcodeScanner] Link '" << this->linkName
                  << "' not found in model '" << this->model.Name(_ecm)
                  << "' (will keep trying). Available links:" << available
                  << "\n";
          this->linkWarned = true;
        }
        if (this->publishTf)
          this->tfPub.Publish(tfMsg);
        return;
      }
    }

    // A released or latched LS2208 emits no scan report at all. Silence on the
    // report topic means "not scanning"; an empty "decoded" list means
    // "scanning and resolving nothing". Downstream must distinguish them.
    if (!held || isLatched)
    {
      if (this->publishTf)
        this->tfPub.Publish(tfMsg);
      return;
    }

    const ignition::math::Pose3d sWorld =
        ignition::gazebo::worldPose(this->link, _ecm) * this->mountPose;

    // Counts scans that actually ran, so the "scan" index in consecutive
    // reports is contiguous rather than skipping over released cycles.
    const size_t thisScan = this->scanCount;
    ++this->scanCount;

    // ---- evaluate every label, keep the single best ------------------------
    struct Candidate
    {
      std::string payload;
      std::string name;
      double range{0.0};
      double roll{0.0};
      double pitch{0.0};
      double skew{0.0};
      double margin{0.0};
      ignition::math::Pose3d relPose;
      bool valid{false};
    };
    Candidate best;

    // Contrast is a global property of the printed stock here. Fail fast:
    // below the LS2208's 20% MRD nothing resolves at any geometry.
    const bool contrastOk = (this->labelContrast >= this->printContrast);

    if (contrastOk)
    {
      for (const auto &[labelEntity, label] : this->labels)
      {
        const ignition::math::Pose3d lWorld =
            ignition::gazebo::worldPose(labelEntity, _ecm);

        // Label position in the scanner frame. Boresight +X.
        const ignition::math::Vector3d relPos =
            sWorld.Rot().RotateVectorReverse(lWorld.Pos() - sWorld.Pos());

        const double range = relPos.Length();
        if (range < this->dofNear || range > this->dofFar)
          continue;                              // outside depth of field
        if (relPos.X() <= 0.0)
          continue;                              // behind the exit window

        // Inside the swept line. The sweep is horizontal (scanner XY), so the
        // horizontal gate is the scan angle but the vertical gate is the bar
        // height: a single line has no vertical extent beyond the beam spot.
        const double thetaH = std::atan2(relPos.Y(), relPos.X());
        if (std::abs(thetaH) > this->scanHalfAngle)
          continue;

        const double vTol = std::atan2(
            0.5 * (this->symbolHeight + this->beamSpot), range);
        const double thetaV = std::atan2(
            relPos.Z(),
            std::sqrt(relPos.X() * relPos.X() + relPos.Y() * relPos.Y()));
        if (std::abs(thetaV) > vTol)
          continue;    // line passes above or below the bars

        // Orientation of the label relative to the scanner. The label's
        // outward normal is its local -Y; the symbol's long axis is local +X.
        const ignition::math::Quaterniond relRot =
            sWorld.Rot().Inverse() * lWorld.Rot();
        const ignition::math::Vector3d nrm =
            relRot.RotateVector(-ignition::math::Vector3d::UnitY);
        const ignition::math::Vector3d axis =
            relRot.RotateVector(ignition::math::Vector3d::UnitX);

        // Line of sight must strike the printed face, not its back.
        const ignition::math::Vector3d los = relPos.Normalized();
        const double cosInc = -los.Dot(nrm);
        if (cosInc <= 0.0)
          continue;                              // looking at the back

        const double incidence = std::acos(std::clamp(cosInc, -1.0, 1.0));
        if (incidence < this->specularDeadzone)
          continue;                              // mirror glare straight back

        // Decompose incidence into the datasheet's two axes. Pitch tilts
        // about the symbol's long axis, skew about the bar-height axis.
        // nrm x axis, not axis x nrm: with nrm = label -Y and axis = label +X
        // the former is +Z, the actual bar-height direction. The latter is -Z
        // and silently flips the sign of every reported pitch.
        const ignition::math::Vector3d hgt = nrm.Cross(axis).Normalized();
        const double skew  = std::atan2(los.Dot(axis), cosInc);
        const double pitch = std::atan2(los.Dot(hgt), cosInc);
        if (std::abs(skew) > this->skewLimit)
          continue;
        if (std::abs(pitch) > this->pitchLimit)
          continue;

        // Roll: angle between the swept line and the symbol's long axis,
        // measured in the label plane. This is the axis the short bars punish.
        //
        // The trace is the INTERSECTION of the scan plane with the label plane,
        // not a projection of any single ray. Every ray of the sweep lies in
        // the scanner z = 0 plane (boresight +X, sweep in XY), so the whole
        // family of rays meets the label plane along one line, whose direction
        // is Zhat_scanner x nrm. This is exact.
        //
        // The previous form projected scanner +Y into the label plane. That
        // agrees with the line above only when nrm_y or nrm_z is zero — i.e.
        // pure skew or pure pitch, which is why it survived single-axis
        // testing. With BOTH nonzero the projected vector leaves the scan
        // plane and the error is atan2-scale, not second order:
        //     pitch 30 deg, skew 10 deg  ->  5.1 deg of roll error
        //     pitch 65 deg, skew 10 deg  ->  9.1 deg
        // Against the 6.52 deg <auto_roll_limit> that exceeds the entire
        // budget, so the gate mis-decided in both directions and every
        // combined-axis mRoll was wrong with it.
        const ignition::math::Vector3d lineInPlane =
            ignition::math::Vector3d::UnitZ.Cross(nrm);
        // Degenerate only if nrm is parallel to scanner Z — a label facing
        // straight up or down, where the scan plane never crosses it. The
        // cosInc <= 0 test above already turns away most of that case.
        double roll = 0.0;
        if (lineInPlane.Length() > 1e-9)
        {
          const ignition::math::Vector3d lp = lineInPlane.Normalized();
          roll = std::atan2(lp.Dot(hgt), lp.Dot(axis));
          // The symbol is bi-directional and has no head/tail preference, so
          // fold to [-90, 90].
          if (roll > M_PI_2)
            roll -= M_PI;
          else if (roll < -M_PI_2)
            roll += M_PI;
        }
        if (std::abs(roll) > this->rollLimit)
          continue;

        // ---- decode margin ------------------------------------------------
        // Product of how comfortably each gate was cleared. 1.0 is a dead-on
        // presentation at mid-DOF; 0.0 is right at a limit. Real scanners are
        // close to binary inside their envelope, so this is deliberately
        // generous in the middle and only bites near the edges.
        const double mid = 0.5 * (this->dofNear + this->dofFar);
        const double halfSpan = 0.5 * (this->dofFar - this->dofNear);
        const double mRange = halfSpan > 1e-9
            ? 1.0 - std::pow(std::abs(range - mid) / halfSpan, 4.0) : 1.0;
        const double mRoll  = 1.0 - std::pow(
            std::abs(roll) / std::max(this->rollLimit, 1e-9), 2.0);
        const double mSkew  = 1.0 - std::pow(
            std::abs(skew) / std::max(this->skewLimit, 1e-9), 4.0);
        const double mPitch = 1.0 - std::pow(
            std::abs(pitch) / std::max(this->pitchLimit, 1e-9), 4.0);
        const double mVert  = 1.0 - std::pow(
            std::abs(thetaV) / std::max(vTol, 1e-9), 2.0);

        const double margin = std::clamp(
            mRange * mRoll * mSkew * mPitch * mVert, 0.0, 1.0);

        if (best.valid && margin <= best.margin)
          continue;

        // Occlusion is the most expensive test, so it runs last and only for
        // a candidate that would actually win.
        if (!this->occluders.empty() &&
            this->Occluded(sWorld.Pos(), lWorld.Pos(), _ecm))
          continue;

        best.payload = label.payload;
        best.name    = label.name;
        best.range   = range;
        best.roll    = roll;
        best.pitch   = pitch;
        best.skew    = skew;
        best.margin  = margin;
        best.relPose = ignition::math::Pose3d(relPos, relRot);
        best.valid   = true;
      }
    }

    // A scan that clears every gate still fails sometimes; the mirror may be
    // mid-sweep, the spot may straddle a module edge. margin drives it.
    bool decoded = false;
    if (best.valid)
    {
      std::normal_distribution<double> jitter(0.0, this->marginNoise);
      const double p = std::clamp(
          best.margin + (this->marginNoise > 0.0 ? jitter(this->rng) : 0.0),
          0.0, this->pDecodeMax);
      std::uniform_real_distribution<double> uniform(0.0, 1.0);
      decoded = (uniform(this->rng) < p);
    }

    // ---- JSON scan report ---------------------------------------------------
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{\"t\":%.6f,\"scan\":%zu,\"scanner\":\"%s\",\"mode\":\"%s\","
        "\"trigger\":true,\"decoded\":[",
        simTime.count(), thisScan, this->scannerId.c_str(),
        pres ? "presentation" : "trigger");
    std::string json = buf;

    if (decoded)
    {
      std::snprintf(buf, sizeof(buf),
          "{\"data\":\"%s\",\"symbology\":\"CODE128\",\"r\":%.4f,"
          "\"roll\":%.2f,\"pitch\":%.2f,\"skew\":%.2f,\"margin\":%.3f}",
          best.payload.c_str(), best.range,
          best.roll  * 180.0 / M_PI,
          best.pitch * 180.0 / M_PI,
          best.skew  * 180.0 / M_PI,
          best.margin);
      json += buf;
    }
    json += "]";
    // Ground truth with no hardware counterpart — validation only.
    json += ",\"n_labels\":" + std::to_string(this->labels.size());
    json += "}";

    ignition::msgs::StringMsg msg;
    msg.set_data(json);
    this->pub.Publish(msg);

    if (decoded)
    {
      {
        // Same mutex as the read above: OnTriggerCmd clears latched on a
        // rising edge from the transport thread.
        std::lock_guard<std::mutex> lock(this->triggerMutex);
        this->latched = true;               // one decode per trigger pull
        this->lastDecodeTime = simTime.count();
      }
      if (this->publishTf)
      {
        this->AddTransform(tfMsg,
            "barcode_scanner_" + this->scannerId, best.name,
            best.relPose, simTime);
      }
    }

    if (this->publishTf)
      this->tfPub.Publish(tfMsg);
  }

  /// \brief Append one stamped transform, using the frame_id /
  /// child_frame_id header-data convention ros_gz_bridge's
  /// Pose_V <-> tf2_msgs/TFMessage conversion expects.
  private: void AddTransform(ignition::msgs::Pose_V &_msg,
      const std::string &_frame, const std::string &_child,
      const ignition::math::Pose3d &_pose,
      const std::chrono::duration<double> &_t) const
  {
    auto *pose = _msg.add_pose();
    pose->set_name(_child);

    auto *header = pose->mutable_header();
    const auto sec = static_cast<int64_t>(_t.count());
    header->mutable_stamp()->set_sec(sec);
    header->mutable_stamp()->set_nsec(
        static_cast<int32_t>((_t.count() - static_cast<double>(sec)) * 1e9));

    auto *frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(_frame);
    auto *child = header->add_data();
    child->set_key("child_frame_id");
    child->add_value(_child);

    ignition::msgs::Set(pose, _pose);
  }

  // ---- state ------------------------------------------------------------
  private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  private: std::unordered_map<ignition::gazebo::Entity, LabelInfo> labels;
  private: std::vector<Occluder> occluders;
  private: bool labelsSeeded{false};
  private: size_t scanCount{0};

  private: std::string scannerId{"ls2208"};
  private: std::string labelPrefix{"barcode_label_"};
  private: std::string linkName{"arm0_hand"};
  private: std::string parentFrame{"arm0_hand"};
  private: ignition::math::Pose3d mountPose;
  private: ignition::gazebo::Entity link{ignition::gazebo::kNullEntity};
  private: bool linkWarned{false};

  private: double updateRate{100.0};       ///< scans/sec, datasheet typical

  // Depth of field defaults for OUR 28 mm / 8.35 mil symbol. Neither is a
  // datasheet cell: dofFar is interpolated between the 10/2015 sheet's 7.5 mil
  // and 13 mil rows, and dofNear is the scan-line-width floor 0.028/0.6306,
  // not the sheet's 0.1". See the header block before changing either.
  private: double dofNear{0.04440};
  private: double dofFar{0.50873};

  private: double symbolWidth{0.028};      ///< printed extent incl. quiet zones
  private: double symbolHeight{0.003};     ///< bar height
  private: double beamSpot{0.0002};        ///< spot diameter at focus

  private: double scanHalfAngle{17.5 * M_PI / 180.0};
  private: bool autoRollLimit{true};
  private: double rollLimit{60.0 * M_PI / 180.0};
  private: double pitchLimit{65.0 * M_PI / 180.0};
  private: double skewLimit{10.0 * M_PI / 180.0};
  private: double specularDeadzone{0.0};

  private: double printContrast{0.20};     ///< datasheet minimum MRD
  private: double labelContrast{0.85};     ///< what the sim's stock achieves

  private: double pDecodeMax{0.99};
  private: double marginNoise{0.05};

  private: bool presentation{false};       ///< Intellistand hands-free
  private: bool triggerHeld{false};
  private: bool latched{false};            ///< one decode per pull
  private: double rescanDelay{0.5};
  private: double lastDecodeTime{-1e9};
  private: std::mutex triggerMutex;        ///< guards triggerHeld/presentation

  private: bool publishTf{true};
  private: unsigned int seedUsed{0};

  private: std::chrono::duration<double> period{0.01};
  private: std::chrono::duration<double> lastUpdate{-1.0};

  private: std::mt19937 rng;
  private: ignition::transport::Node node;
  private: ignition::transport::Node::Publisher pub;
  private: ignition::transport::Node::Publisher tfPub;
};

}  // namespace barcode

IGNITION_ADD_PLUGIN(
    barcode::BarcodeScanner,
    ignition::gazebo::System,
    barcode::BarcodeScanner::ISystemConfigure,
    barcode::BarcodeScanner::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(barcode::BarcodeScanner, "barcode::BarcodeScanner")
