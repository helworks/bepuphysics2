#if HELENGINE_CODEGEN_FEATURE_DISABLED_PHYSICS3D_DIAGNOSTICS
using System.Numerics;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuPhysics {
    /// <summary>
    /// Provides a compact no-op diagnostics surface when the generic physics-diagnostics runtime feature is disabled for code generation.
    /// </summary>
    public static class BepuNativeConversionDiagnostics {
        /// <summary>
        /// Accepts diagnostics reset requests without retaining any buffered trace state when diagnostics are disabled.
        /// </summary>
        /// <param name="enabled">Ignored diagnostics-enabled flag.</param>
        public static void Reset(bool enabled) {
        }

        /// <summary>
        /// Ignores physics-step notifications when diagnostics are disabled.
        /// </summary>
        public static void BeginPhysicsStep() {
        }

        /// <summary>
        /// Discards constrained-kinematic prepass probes when diagnostics are disabled.
        /// </summary>
        public static void RecordKinematicPrepass(bool integratesPoseFirst, int workerIndex, int bodyCount, int bundleStartIndex, int bundleEndIndex, int bundleIndex, int bundleBaseIndex, int countInBundle, Buffer<int> bodyHandles, Vector<int> bodyIndicesVector) {
        }

        /// <summary>
        /// Discards post-substep selection probes when diagnostics are disabled.
        /// </summary>
        public static void RecordPostSubstepSelection(Vector<int> bodyIndices, Vector<int> unconstrainedMask, Vector<int> unconstrainedVelocityIntegrationMask, Vector<int> velocityMaskedBodyIndices, bool anyBodyInBundleIsUnconstrained, bool anyBodyInBundleNeedsVelocityIntegration) {
        }

        /// <summary>
        /// Discards scatter-velocity probes when diagnostics are disabled.
        /// </summary>
        public static void RecordScatterVelocities(BodyVelocityWide sourceVelocities, Vector<int> encodedBodyIndices) {
        }

        /// <summary>
        /// Discards two-body solve probes when diagnostics are disabled.
        /// </summary>
        public static void RecordTwoBodySolveProbe(
            string constraintTypeName,
            string phase,
            Bodies bodies,
            int bundleIndex,
            Vector<int> indexA,
            Vector<int> indexB,
            Vector3Wide positionA,
            QuaternionWide orientationA,
            Vector3Wide positionB,
            QuaternionWide orientationB,
            BodyVelocityWide wsvA,
            BodyVelocityWide wsvB) {
        }

        /// <summary>
        /// Discards integration-responsibility probes when diagnostics are disabled.
        /// </summary>
        public static void RecordIntegrationResponsibilityProbe(string phase, int batchIndex, int typeBatchIndex, int bodyIndexInConstraint, int constraintIndex, int bodyHandle, bool isFirstObservedInBatch) {
        }

        /// <summary>
        /// Discards integration batch-merge probes when diagnostics are disabled.
        /// </summary>
        public static void RecordIntegrationResponsibilityBatchMergeProbe(int batchIndex, int flagBundleCount, int scalarLoopStartIndex, bool hasAnyIntegrationResponsibilities) {
        }

        /// <summary>
        /// Discards integration type-batch probes when diagnostics are disabled.
        /// </summary>
        public static void RecordIntegrationResponsibilityTypeBatchProbe(int batchIndex, int typeBatchIndex, int constraintStart, int exclusiveConstraintEnd, int constraintCount, int bundleCount, int bodiesPerConstraint, int bundleStartIndex, int bundleEndIndex) {
        }

        /// <summary>
        /// Reports that no tracked stack body is being monitored when diagnostics are disabled.
        /// </summary>
        /// <param name="bodyHandle">Ignored body handle.</param>
        /// <returns>Always <c>false</c>.</returns>
        public static bool ShouldRecordTrackedStackBodyHandle(int bodyHandle) {
            return false;
        }

        /// <summary>
        /// Discards gather-and-integrate probes when diagnostics are disabled.
        /// </summary>
        public static void RecordGatherAndIntegrateProbe(
            string phase,
            Bodies bodies,
            int bundleIndex,
            int bodyIndexInConstraint,
            Vector<int> encodedBodyIndices,
            Vector3Wide position,
            QuaternionWide orientation,
            BodyVelocityWide velocity) {
        }

        /// <summary>
        /// Returns an empty buffered snapshot payload when diagnostics are disabled.
        /// </summary>
        /// <returns>Always <see cref="string.Empty"/>.</returns>
        public static string DrainPendingText() {
            return string.Empty;
        }
    }
}
#else
using System.Numerics;
using System.Text;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuPhysics {
    /// <summary>
    /// Captures bounded native-conversion diagnostics for reduced-slice BEPU debugging.
    /// </summary>
    public static class BepuNativeConversionDiagnostics {
        /// <summary>
        /// Maximum number of scatter snapshots recorded for one reset window.
        /// </summary>
        const int MaxScatterVelocitySnapshots = 16;

        /// <summary>
        /// Maximum number of post-substep selection snapshots recorded for one reset window.
        /// </summary>
        const int MaxPostSubstepSelectionSnapshots = 24;

        /// <summary>
        /// Maximum number of constrained-kinematic prepass snapshots recorded for one reset window.
        /// </summary>
        const int MaxKinematicPrepassSnapshots = 16;

        /// <summary>
        /// Maximum number of two-body solve snapshots recorded for one reset window.
        /// </summary>
        const int MaxTwoBodySolveSnapshots = 24;

        /// <summary>
        /// Maximum number of integration-responsibility snapshots recorded for one reset window.
        /// </summary>
        const int MaxIntegrationResponsibilitySnapshots = 32;

        /// <summary>
        /// Maximum number of integration-responsibility batch-merge probes recorded for one reset window.
        /// </summary>
        const int MaxIntegrationResponsibilityBatchMergeSnapshots = 16;

        /// <summary>
        /// Maximum number of integration-responsibility type-batch entry probes recorded for one reset window.
        /// </summary>
        const int MaxIntegrationResponsibilityTypeBatchSnapshots = 24;

        /// <summary>
        /// Stores the fixed decimal scale used to serialize floating-point values without culture-sensitive formatting APIs.
        /// </summary>
        const long TraceFloatScale = 1000000000L;

        /// <summary>
        /// Gets or sets a value indicating whether scatter diagnostics should be recorded.
        /// </summary>
        static bool IsEnabled { get; set; }

        /// <summary>
        /// Gets or sets the number of scatter snapshots already recorded for the current reset window.
        /// </summary>
        static int ScatterVelocitySnapshotCount { get; set; }

        /// <summary>
        /// Gets or sets the number of post-substep selection snapshots already recorded for the current reset window.
        /// </summary>
        static int PostSubstepSelectionSnapshotCount { get; set; }

        /// <summary>
        /// Gets or sets the number of constrained-kinematic prepass snapshots already recorded for the current reset window.
        /// </summary>
        static int KinematicPrepassSnapshotCount { get; set; }

        /// <summary>
        /// Gets or sets the number of two-body solve snapshots already recorded for the current reset window.
        /// </summary>
        static int TwoBodySolveSnapshotCount { get; set; }

        /// <summary>
        /// Gets or sets the number of integration-responsibility snapshots already recorded for the current reset window.
        /// </summary>
        static int IntegrationResponsibilitySnapshotCount { get; set; }

        /// <summary>
        /// Gets or sets the number of integration-responsibility batch-merge probes already recorded for the current reset window.
        /// </summary>
        static int IntegrationResponsibilityBatchMergeSnapshotCount { get; set; }

        /// <summary>
        /// Gets or sets the number of integration-responsibility type-batch probes already recorded for the current reset window.
        /// </summary>
        static int IntegrationResponsibilityTypeBatchSnapshotCount { get; set; }

        /// <summary>
        /// Gets or sets the zero-based fixed-step frame index shared with the managed reduced-BEPU diagnostics path.
        /// </summary>
        static int CurrentSimulationFrameIndex { get; set; }

        /// <summary>
        /// Stores pending scatter diagnostics until the host consumes them.
        /// </summary>
        static string PendingSnapshotText { get; set; } = string.Empty;

        /// <summary>
        /// Resets the bounded diagnostic state for one new traced run.
        /// </summary>
        /// <param name="enabled">Controls whether subsequent diagnostics are recorded.</param>
        public static void Reset(bool enabled) {
            IsEnabled = enabled;
            ScatterVelocitySnapshotCount = 0;
            PostSubstepSelectionSnapshotCount = 0;
            KinematicPrepassSnapshotCount = 0;
            TwoBodySolveSnapshotCount = 0;
            IntegrationResponsibilitySnapshotCount = 0;
            IntegrationResponsibilityBatchMergeSnapshotCount = 0;
            IntegrationResponsibilityTypeBatchSnapshotCount = 0;
            CurrentSimulationFrameIndex = -1;
            PendingSnapshotText = string.Empty;
        }

        /// <summary>
        /// Advances the shared fixed-step frame index at the start of each traced physics step.
        /// </summary>
        public static void BeginPhysicsStep() {
            if (!IsEnabled) {
                return;
            }

            CurrentSimulationFrameIndex++;
        }

        /// <summary>
        /// Records one bounded constrained-kinematic prepass bundle snapshot so dynamic-body contamination can be compared across runtimes.
        /// </summary>
        /// <param name="integratesPoseFirst">Indicates whether this prepass integrates pose before velocity.</param>
        /// <param name="workerIndex">Worker responsible for the current prepass bundle.</param>
        /// <param name="bodyCount">Total body-handle count in the constrained kinematic list.</param>
        /// <param name="bundleStartIndex">Inclusive bundle start for the current prepass dispatch.</param>
        /// <param name="bundleEndIndex">Exclusive bundle end for the current prepass dispatch.</param>
        /// <param name="bundleIndex">Current bundle index inside the prepass loop.</param>
        /// <param name="bundleBaseIndex">Base handle index for the current bundle.</param>
        /// <param name="countInBundle">Number of valid lanes in the current bundle.</param>
        /// <param name="bodyHandles">Constrained kinematic body handles passed into the prepass.</param>
        /// <param name="bodyIndicesVector">Resolved active-set body indices used for gather/scatter.</param>
        public static void RecordKinematicPrepass(bool integratesPoseFirst, int workerIndex, int bodyCount, int bundleStartIndex, int bundleEndIndex, int bundleIndex, int bundleBaseIndex, int countInBundle, Buffer<int> bodyHandles, Vector<int> bodyIndicesVector) {
            if (!IsEnabled || KinematicPrepassSnapshotCount >= MaxKinematicPrepassSnapshots) {
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.Append("[BepuKinematicPrepass]");
            builder.Append(" frame=");
            builder.Append(KinematicPrepassSnapshotCount);
            builder.Append(" stage=");
            builder.Append(integratesPoseFirst ? "posevel" : "vel");
            builder.Append(" worker=");
            builder.Append(workerIndex);
            builder.Append(" bodyCount=");
            builder.Append(bodyCount);
            builder.Append(" bundleRange=");
            builder.Append(bundleStartIndex);
            builder.Append("..");
            builder.Append(bundleEndIndex);
            builder.Append(" bundleIndex=");
            builder.Append(bundleIndex);
            builder.Append(" countInBundle=");
            builder.Append(countInBundle);
            for (int innerIndex = 0; innerIndex < Vector<int>.Count; innerIndex++) {
                builder.Append(" lane");
                builder.Append(innerIndex);
                builder.Append("={handle=");
                if (innerIndex < countInBundle) {
                    builder.Append(bodyHandles[bundleBaseIndex + innerIndex]);
                }
                else {
                    builder.Append(-1);
                }

                builder.Append(",body=");
                builder.Append(bodyIndicesVector[innerIndex]);
                builder.Append("}");
            }

            builder.AppendLine();
            PendingSnapshotText += builder.ToString();
            KinematicPrepassSnapshotCount++;
        }

        /// <summary>
        /// Records one bounded post-substep body-selection snapshot before velocity integration executes.
        /// </summary>
        /// <param name="bodyIndices">Bundle body indices for the current post-substep pass.</param>
        /// <param name="unconstrainedMask">Mask indicating which lanes are unconstrained.</param>
        /// <param name="unconstrainedVelocityIntegrationMask">Mask indicating which lanes should receive velocity integration.</param>
        /// <param name="velocityMaskedBodyIndices">Body indices after disabled lanes have been masked to ignored values.</param>
        public static void RecordPostSubstepSelection(Vector<int> bodyIndices, Vector<int> unconstrainedMask, Vector<int> unconstrainedVelocityIntegrationMask, Vector<int> velocityMaskedBodyIndices, bool anyBodyInBundleIsUnconstrained, bool anyBodyInBundleNeedsVelocityIntegration) {
            if (!IsEnabled || PostSubstepSelectionSnapshotCount >= MaxPostSubstepSelectionSnapshots) {
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.Append("[BepuPostSubstepSelection]");
            builder.Append(" frame=");
            builder.Append(PostSubstepSelectionSnapshotCount);
            builder.Append(" anyUnconstrained=");
            builder.Append(anyBodyInBundleIsUnconstrained ? 1 : 0);
            builder.Append(" anyVelocityIntegration=");
            builder.Append(anyBodyInBundleNeedsVelocityIntegration ? 1 : 0);
            for (int innerIndex = 0; innerIndex < Vector<int>.Count; innerIndex++) {
                builder.Append(" lane");
                builder.Append(innerIndex);
                builder.Append("={body=");
                builder.Append(bodyIndices[innerIndex]);
                builder.Append(",unconstrained=");
                builder.Append(unconstrainedMask[innerIndex] < 0 ? 1 : 0);
                builder.Append(",velocityIntegrates=");
                builder.Append(unconstrainedVelocityIntegrationMask[innerIndex] < 0 ? 1 : 0);
                builder.Append(",maskedBody=");
                builder.Append(velocityMaskedBodyIndices[innerIndex]);
                builder.Append("}");
            }

            builder.AppendLine();
            PendingSnapshotText += builder.ToString();
            PostSubstepSelectionSnapshotCount++;
        }

        /// <summary>
        /// Records one bounded scatter-velocity snapshot using the current encoded indices and wide velocities.
        /// </summary>
        /// <param name="sourceVelocities">Wide source velocities about to be written back into body state.</param>
        /// <param name="encodedBodyIndices">Per-lane encoded body references used for the writeback.</param>
        public static void RecordScatterVelocities(BodyVelocityWide sourceVelocities, Vector<int> encodedBodyIndices) {
            if (!IsEnabled || ScatterVelocitySnapshotCount >= MaxScatterVelocitySnapshots) {
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.Append("[BepuScatterVelocities]");
            builder.Append(" frame=");
            builder.Append(ScatterVelocitySnapshotCount);
            for (int innerIndex = 0; innerIndex < Vector<int>.Count; innerIndex++) {
                builder.Append(" lane");
                builder.Append(innerIndex);
                builder.Append("={encoded=");
                builder.Append(encodedBodyIndices[innerIndex]);
                builder.Append(",active=");
                builder.Append((uint)encodedBodyIndices[innerIndex] < Bodies.DynamicLimit ? 1 : 0);
                builder.Append(",linearXScaled=");
                builder.Append((int)(sourceVelocities.Linear.X[innerIndex] * 1000000.0f));
                builder.Append(",linearYScaled=");
                builder.Append((int)(sourceVelocities.Linear.Y[innerIndex] * 1000000.0f));
                builder.Append(",linearZScaled=");
                builder.Append((int)(sourceVelocities.Linear.Z[innerIndex] * 1000000.0f));
                builder.Append("}");
            }

            builder.AppendLine();
            PendingSnapshotText += builder.ToString();
            ScatterVelocitySnapshotCount++;
        }

        /// <summary>
        /// Records one bounded two-body solve snapshot for bundles that include the tracked second stack body.
        /// </summary>
        /// <param name="constraintTypeName">Constraint-function type name associated with the current solve bundle.</param>
        /// <param name="phase">Logical phase name for the recorded snapshot.</param>
        /// <param name="bodies">Body collection used to resolve tracked handles from encoded references.</param>
        /// <param name="bundleIndex">Current type-batch bundle index being solved.</param>
        /// <param name="indexA">Encoded body references for side A.</param>
        /// <param name="indexB">Encoded body references for side B.</param>
        /// <param name="positionA">Current side-A gathered positions.</param>
        /// <param name="orientationA">Current side-A gathered orientations.</param>
        /// <param name="positionB">Current side-B gathered positions.</param>
        /// <param name="orientationB">Current side-B gathered orientations.</param>
        /// <param name="wsvA">Current side-A wide velocities.</param>
        /// <param name="wsvB">Current side-B wide velocities.</param>
        public static void RecordTwoBodySolveProbe(
            string constraintTypeName,
            string phase,
            Bodies bodies,
            int bundleIndex,
            Vector<int> indexA,
            Vector<int> indexB,
            Vector3Wide positionA,
            QuaternionWide orientationA,
            Vector3Wide positionB,
            QuaternionWide orientationB,
            BodyVelocityWide wsvA,
            BodyVelocityWide wsvB) {
            if (!IsEnabled || TwoBodySolveSnapshotCount >= MaxTwoBodySolveSnapshots) {
                return;
            }

            bool containsTrackedBody = false;
            for (int laneIndex = 0; laneIndex < Vector<int>.Count; laneIndex++) {
                if (indexA[laneIndex] == 1 || indexB[laneIndex] == 1) {
                    containsTrackedBody = true;
                    break;
                }
            }

            if (!containsTrackedBody) {
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.Append("[BepuTwoBodySolve]");
            builder.Append(" frame=");
            builder.Append(TwoBodySolveSnapshotCount);
            builder.Append(" type=");
            builder.Append(constraintTypeName);
            builder.Append(" phase=");
            builder.Append(phase);
            builder.Append(" bundle=");
            builder.Append(bundleIndex);
            AppendTwoBodySolveSide(builder, "A", indexA, wsvA);
            AppendTwoBodySolveSide(builder, "B", indexB, wsvB);
            builder.AppendLine();
            PendingSnapshotText += builder.ToString();
            RecordTwoBodySolveStructuredPhase(phase, bodies, bundleIndex, indexA, positionA, orientationA, wsvA);
            RecordTwoBodySolveStructuredPhase(phase, bodies, bundleIndex, indexB, positionB, orientationB, wsvB);
            TwoBodySolveSnapshotCount++;
        }

        /// <summary>
        /// Records one bounded integration-responsibility snapshot for the tracked stack body handle.
        /// </summary>
        /// <param name="phase">Logical phase name describing the current prepass event.</param>
        /// <param name="batchIndex">Constraint batch index currently being processed.</param>
        /// <param name="typeBatchIndex">Type-batch index currently being processed.</param>
        /// <param name="bodyIndexInConstraint">Body slot within the current constraint type.</param>
        /// <param name="constraintIndex">Constraint index inside the current type batch.</param>
        /// <param name="bodyHandle">Tracked body handle observed at this prepass point.</param>
        /// <param name="isFirstObservedInBatch">Indicates whether the tracked body belongs to the first-observed set for the current batch.</param>
        public static void RecordIntegrationResponsibilityProbe(string phase, int batchIndex, int typeBatchIndex, int bodyIndexInConstraint, int constraintIndex, int bodyHandle, bool isFirstObservedInBatch) {
            if (!IsEnabled || IntegrationResponsibilitySnapshotCount >= MaxIntegrationResponsibilitySnapshots) {
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.Append("[BepuIntegrationResponsibility]");
            builder.Append(" frame=");
            builder.Append(IntegrationResponsibilitySnapshotCount);
            builder.Append(" phase=");
            builder.Append(phase);
            builder.Append(" batch=");
            builder.Append(batchIndex);
            builder.Append(" typeBatch=");
            builder.Append(typeBatchIndex);
            builder.Append(" bodySlot=");
            builder.Append(bodyIndexInConstraint);
            builder.Append(" constraint=");
            builder.Append(constraintIndex);
            builder.Append(" handle=");
            builder.Append(bodyHandle);
            builder.Append(" firstObserved=");
            builder.Append(isFirstObservedInBatch ? 1 : 0);
            builder.AppendLine();
            PendingSnapshotText += builder.ToString();
            if (ShouldRecordTrackedBodyHandle(bodyHandle)) {
                PendingSnapshotText += BuildStructuredTraceRecord(
                    "integration_responsibility_assignment",
                    bodyHandle,
                    -1,
                    -1,
                    batchIndex,
                    typeBatchIndex,
                    bodyIndexInConstraint,
                    string.Empty,
                    phase,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f,
                    1f,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f);
            }

            IntegrationResponsibilitySnapshotCount++;
        }

        /// <summary>
        /// Records one bounded batch-merge summary for the integration-responsibility prepass.
        /// </summary>
        /// <param name="batchIndex">Constraint batch whose first-observed merge just completed.</param>
        /// <param name="flagBundleCount">Number of flag bundles examined for the batch.</param>
        /// <param name="scalarLoopStartIndex">Index where the scalar merge tail began.</param>
        /// <param name="hasAnyIntegrationResponsibilities">Indicates whether any first-observed bits were found for the batch.</param>
        public static void RecordIntegrationResponsibilityBatchMergeProbe(int batchIndex, int flagBundleCount, int scalarLoopStartIndex, bool hasAnyIntegrationResponsibilities) {
            if (!IsEnabled || IntegrationResponsibilityBatchMergeSnapshotCount >= MaxIntegrationResponsibilityBatchMergeSnapshots) {
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.Append("[BepuIntegrationBatchMerge]");
            builder.Append(" frame=");
            builder.Append(CurrentSimulationFrameIndex);
            builder.Append(" batch=");
            builder.Append(batchIndex);
            builder.Append(" flagBundleCount=");
            builder.Append(flagBundleCount);
            builder.Append(" scalarLoopStart=");
            builder.Append(scalarLoopStartIndex);
            builder.Append(" hasAny=");
            builder.Append(hasAnyIntegrationResponsibilities ? 1 : 0);
            builder.AppendLine();
            PendingSnapshotText += builder.ToString();
            IntegrationResponsibilityBatchMergeSnapshotCount++;
        }

        /// <summary>
        /// Records one bounded type-batch entry for the integration-responsibility prepass.
        /// </summary>
        /// <param name="batchIndex">Constraint batch being evaluated.</param>
        /// <param name="typeBatchIndex">Type batch being evaluated.</param>
        /// <param name="constraintStart">Inclusive constraint start index for the region.</param>
        /// <param name="exclusiveConstraintEnd">Exclusive constraint end index for the region.</param>
        /// <param name="constraintCount">Total number of constraints in the current type batch.</param>
        /// <param name="bundleCount">Total number of bundles in the current type batch.</param>
        /// <param name="bodiesPerConstraint">Number of bodies referenced by each constraint in the current type batch.</param>
        /// <param name="bundleStartIndex">Inclusive bundle start index for the region.</param>
        /// <param name="bundleEndIndex">Exclusive bundle end index for the region.</param>
        public static void RecordIntegrationResponsibilityTypeBatchProbe(int batchIndex, int typeBatchIndex, int constraintStart, int exclusiveConstraintEnd, int constraintCount, int bundleCount, int bodiesPerConstraint, int bundleStartIndex, int bundleEndIndex) {
            if (!IsEnabled || IntegrationResponsibilityTypeBatchSnapshotCount >= MaxIntegrationResponsibilityTypeBatchSnapshots) {
                return;
            }

            StringBuilder builder = new StringBuilder();
            builder.Append("[BepuIntegrationTypeBatch]");
            builder.Append(" frame=");
            builder.Append(CurrentSimulationFrameIndex);
            builder.Append(" batch=");
            builder.Append(batchIndex);
            builder.Append(" typeBatch=");
            builder.Append(typeBatchIndex);
            builder.Append(" start=");
            builder.Append(constraintStart);
            builder.Append(" end=");
            builder.Append(exclusiveConstraintEnd);
            builder.Append(" constraintCount=");
            builder.Append(constraintCount);
            builder.Append(" bundleCount=");
            builder.Append(bundleCount);
            builder.Append(" bodiesPerConstraint=");
            builder.Append(bodiesPerConstraint);
            builder.Append(" bundleStart=");
            builder.Append(bundleStartIndex);
            builder.Append(" bundleEnd=");
            builder.Append(bundleEndIndex);
            builder.AppendLine();
            PendingSnapshotText += builder.ToString();
            IntegrationResponsibilityTypeBatchSnapshotCount++;
        }

        /// <summary>
        /// Gets whether one body handle belongs to the tracked reduced stack-box diagnostic set.
        /// </summary>
        /// <param name="bodyHandle">Body handle to evaluate.</param>
        /// <returns><c>true</c> when the handle belongs to the tracked set; otherwise <c>false</c>.</returns>
        public static bool ShouldRecordTrackedStackBodyHandle(int bodyHandle) {
            return ShouldRecordTrackedBodyHandle(bodyHandle);
        }

        /// <summary>
        /// Records one structured gather-and-integrate probe for tracked handles in the current bundle.
        /// </summary>
        /// <param name="phase">Shared-schema phase token for the probe.</param>
        /// <param name="bodies">Body collection used to resolve body handles for active lanes.</param>
        /// <param name="bundleIndex">Current bundle index.</param>
        /// <param name="bodyIndexInConstraint">Body slot within the active constraint type.</param>
        /// <param name="encodedBodyIndices">Encoded body references used by the gather step.</param>
        /// <param name="position">Gathered positions for the current bundle.</param>
        /// <param name="orientation">Gathered orientations for the current bundle.</param>
        /// <param name="velocity">Gathered or integrated velocities for the current bundle.</param>
        public static void RecordGatherAndIntegrateProbe(
            string phase,
            Bodies bodies,
            int bundleIndex,
            int bodyIndexInConstraint,
            Vector<int> encodedBodyIndices,
            Vector3Wide position,
            QuaternionWide orientation,
            BodyVelocityWide velocity) {
            if (!IsEnabled) {
                return;
            }
            if (bodies == null) {
                throw new System.ArgumentNullException(nameof(bodies));
            }

            for (int laneIndex = 0; laneIndex < Vector<int>.Count; laneIndex++) {
                if (!TryGetTrackedBodyReference(bodies, encodedBodyIndices[laneIndex], out int bodyHandle, out int bodyIndex)) {
                    continue;
                }

                PendingSnapshotText += BuildStructuredTraceRecord(
                    phase,
                    bodyHandle,
                    bodyIndex,
                    bundleIndex,
                    -1,
                    -1,
                    laneIndex,
                    FormatIntVector(encodedBodyIndices),
                    string.Empty,
                    position.X[laneIndex],
                    position.Y[laneIndex],
                    position.Z[laneIndex],
                    orientation.X[laneIndex],
                    orientation.Y[laneIndex],
                    orientation.Z[laneIndex],
                    orientation.W[laneIndex],
                    velocity.Linear.X[laneIndex],
                    velocity.Linear.Y[laneIndex],
                    velocity.Linear.Z[laneIndex],
                    velocity.Angular.X[laneIndex],
                    velocity.Angular.Y[laneIndex],
                    velocity.Angular.Z[laneIndex]);
            }
        }

        /// <summary>
        /// Returns all pending snapshot text and clears the buffered diagnostics.
        /// </summary>
        /// <returns>Buffered scatter diagnostics or an empty string when no diagnostics are pending.</returns>
        public static string DrainPendingText() {
            string pendingText = PendingSnapshotText;
            PendingSnapshotText = string.Empty;
            return pendingText;
        }

        /// <summary>
        /// Records one structured two-body solve phase for tracked handles present in one solve side.
        /// </summary>
        /// <param name="phase">Shared-schema phase token for the solve side.</param>
        /// <param name="bodies">Body collection used to resolve tracked handles from encoded references.</param>
        /// <param name="bundleIndex">Current solve bundle index.</param>
        /// <param name="encodedBodyIndices">Encoded body references for one solve side.</param>
        /// <param name="position">Gathered positions for the side.</param>
        /// <param name="orientation">Gathered orientations for the side.</param>
        /// <param name="velocity">Gathered or solved velocities for the side.</param>
        static void RecordTwoBodySolveStructuredPhase(
            string phase,
            Bodies bodies,
            int bundleIndex,
            Vector<int> encodedBodyIndices,
            Vector3Wide position,
            QuaternionWide orientation,
            BodyVelocityWide velocity) {
            if (bodies == null) {
                throw new System.ArgumentNullException(nameof(bodies));
            }

            for (int laneIndex = 0; laneIndex < Vector<int>.Count; laneIndex++) {
                if (!TryGetTrackedBodyReference(bodies, encodedBodyIndices[laneIndex], out int bodyHandle, out int bodyIndex)) {
                    continue;
                }

                PendingSnapshotText += BuildStructuredTraceRecord(
                    phase,
                    bodyHandle,
                    bodyIndex,
                    bundleIndex,
                    -1,
                    -1,
                    laneIndex,
                    FormatIntVector(encodedBodyIndices),
                    string.Empty,
                    position.X[laneIndex],
                    position.Y[laneIndex],
                    position.Z[laneIndex],
                    orientation.X[laneIndex],
                    orientation.Y[laneIndex],
                    orientation.Z[laneIndex],
                    orientation.W[laneIndex],
                    velocity.Linear.X[laneIndex],
                    velocity.Linear.Y[laneIndex],
                    velocity.Linear.Z[laneIndex],
                    velocity.Angular.X[laneIndex],
                    velocity.Angular.Y[laneIndex],
                    velocity.Angular.Z[laneIndex]);
            }
        }

        /// <summary>
        /// Resolves one tracked dynamic body reference from one encoded body index when the lane belongs to the traced four-box stack.
        /// </summary>
        /// <param name="bodies">Body collection used to decode the active-set handle.</param>
        /// <param name="encodedBodyIndex">Encoded body reference from a BEPU bundle lane.</param>
        /// <param name="bodyHandle">Resolved tracked body handle when one is present.</param>
        /// <param name="bodyIndex">Resolved active-set body index when one is present.</param>
        /// <returns>True when the encoded reference points at one tracked dynamic body.</returns>
        static bool TryGetTrackedBodyReference(Bodies bodies, int encodedBodyIndex, out int bodyHandle, out int bodyIndex) {
            bodyHandle = -1;
            bodyIndex = -1;
            if ((uint)encodedBodyIndex >= Bodies.DynamicLimit) {
                return false;
            }

            bodyIndex = encodedBodyIndex & Bodies.BodyReferenceMask;
            bodyHandle = bodies.ActiveSet.IndexToHandle[bodyIndex].Value;
            return ShouldRecordTrackedBodyHandle(bodyHandle);
        }

        /// <summary>
        /// Determines whether one body handle belongs to the tracked reduced stack-box set.
        /// </summary>
        /// <param name="bodyHandle">Body handle to inspect.</param>
        /// <returns>True when the handle belongs to the traced dynamic tower bodies.</returns>
        static bool ShouldRecordTrackedBodyHandle(int bodyHandle) {
            return bodyHandle >= 0 && bodyHandle <= 3;
        }

        /// <summary>
        /// Appends one shared-schema trace line into the pending native diagnostic buffer.
        /// </summary>
        /// <param name="pendingText">Pending trace text receiving the new line.</param>
        /// <param name="phase">Shared-schema phase token.</param>
        /// <param name="bodyHandle">Tracked dynamic body handle.</param>
        /// <param name="bodyIndex">Active-set body index or `-1` when unavailable.</param>
        /// <param name="bundleIndex">Bundle index or `-1` when unavailable.</param>
        /// <param name="constraintBatchIndex">Constraint batch index or `-1` when unavailable.</param>
        /// <param name="typeBatchIndex">Type-batch index or `-1` when unavailable.</param>
        /// <param name="bodySlotIndex">Body slot index or `-1` when unavailable.</param>
        /// <param name="encodedReferences">Encoded body-reference payload or an empty string when unavailable.</param>
        /// <param name="integrationMask">Integration-mask payload or an empty string when unavailable.</param>
        /// <param name="positionX">X position component.</param>
        /// <param name="positionY">Y position component.</param>
        /// <param name="positionZ">Z position component.</param>
        /// <param name="orientationX">X orientation component.</param>
        /// <param name="orientationY">Y orientation component.</param>
        /// <param name="orientationZ">Z orientation component.</param>
        /// <param name="orientationW">W orientation component.</param>
        /// <param name="linearVelocityX">X linear-velocity component.</param>
        /// <param name="linearVelocityY">Y linear-velocity component.</param>
        /// <param name="linearVelocityZ">Z linear-velocity component.</param>
        /// <param name="angularVelocityX">X angular-velocity component.</param>
        /// <param name="angularVelocityY">Y angular-velocity component.</param>
        /// <param name="angularVelocityZ">Z angular-velocity component.</param>
        static string BuildStructuredTraceRecord(
            string phase,
            int bodyHandle,
            int bodyIndex,
            int bundleIndex,
            int constraintBatchIndex,
            int typeBatchIndex,
            int bodySlotIndex,
            string encodedReferences,
            string integrationMask,
            float positionX,
            float positionY,
            float positionZ,
            float orientationX,
            float orientationY,
            float orientationZ,
            float orientationW,
            float linearVelocityX,
            float linearVelocityY,
            float linearVelocityZ,
            float angularVelocityX,
            float angularVelocityY,
            float angularVelocityZ) {
            StringBuilder builder = new StringBuilder();
            builder.Append("frame=");
            builder.Append(GetStructuredTraceFrameIndex());
            builder.Append(" phase=");
            builder.Append(phase);
            builder.Append(" body_handle=");
            builder.Append(bodyHandle);
            builder.Append(" body_index=");
            builder.Append(bodyIndex);
            if (bundleIndex >= 0) {
                builder.Append(" bundle_index=");
                builder.Append(bundleIndex);
            }
            if (constraintBatchIndex >= 0) {
                builder.Append(" constraint_batch=");
                builder.Append(constraintBatchIndex);
            }
            if (typeBatchIndex >= 0) {
                builder.Append(" type_batch=");
                builder.Append(typeBatchIndex);
            }
            if (bodySlotIndex >= 0) {
                builder.Append(" body_slot=");
                builder.Append(bodySlotIndex);
            }
            if (!string.IsNullOrEmpty(encodedReferences)) {
                builder.Append(" encoded_refs=");
                builder.Append(encodedReferences);
            }
            if (!string.IsNullOrEmpty(integrationMask)) {
                builder.Append(" integration_mask=");
                builder.Append(integrationMask);
            }

            builder.Append(" position=(");
            builder.Append(FormatFloat(positionX));
            builder.Append(",");
            builder.Append(FormatFloat(positionY));
            builder.Append(",");
            builder.Append(FormatFloat(positionZ));
            builder.Append(") orientation=(");
            builder.Append(FormatFloat(orientationX));
            builder.Append(",");
            builder.Append(FormatFloat(orientationY));
            builder.Append(",");
            builder.Append(FormatFloat(orientationZ));
            builder.Append(",");
            builder.Append(FormatFloat(orientationW));
            builder.Append(") linear_velocity=(");
            builder.Append(FormatFloat(linearVelocityX));
            builder.Append(",");
            builder.Append(FormatFloat(linearVelocityY));
            builder.Append(",");
            builder.Append(FormatFloat(linearVelocityZ));
            builder.Append(") angular_velocity=(");
            builder.Append(FormatFloat(angularVelocityX));
            builder.Append(",");
            builder.Append(FormatFloat(angularVelocityY));
            builder.Append(",");
            builder.Append(FormatFloat(angularVelocityZ));
            builder.AppendLine(")");
            return builder.ToString();
        }

        /// <summary>
        /// Returns the shared structured-trace frame index for the current native physics step.
        /// </summary>
        /// <returns>Zero-based structured frame index.</returns>
        static int GetStructuredTraceFrameIndex() {
            if (CurrentSimulationFrameIndex < 0) {
                return 0;
            }

            return CurrentSimulationFrameIndex;
        }

        /// <summary>
        /// Formats one encoded-body vector into the compact comma-delimited schema representation.
        /// </summary>
        /// <param name="value">Encoded-body vector to format.</param>
        /// <returns>Comma-delimited vector payload.</returns>
        static string FormatIntVector(Vector<int> value) {
            StringBuilder builder = new StringBuilder();
            for (int laneIndex = 0; laneIndex < Vector<int>.Count; laneIndex++) {
                if (laneIndex > 0) {
                    builder.Append(',');
                }

                builder.Append(value[laneIndex]);
            }

            return builder.ToString();
        }

        /// <summary>
        /// Formats one scalar floating-point value using invariant compact formatting.
        /// </summary>
        /// <param name="value">Value to format.</param>
        /// <returns>Invariant compact scalar text.</returns>
        static string FormatFloat(float value) {
            if (value == 0f) {
                return "0";
            }

            long scaledValue = (long)System.Math.Round(System.Math.Abs((double)value) * TraceFloatScale);
            long wholePart = scaledValue / TraceFloatScale;
            long fractionalPart = scaledValue % TraceFloatScale;
            StringBuilder builder = new StringBuilder();
            if (value < 0f) {
                builder.Append('-');
            }

            builder.Append((int)wholePart);
            if (fractionalPart == 0L) {
                return builder.ToString();
            }

            builder.Append('.');
            AppendFractionDigits(builder, fractionalPart);
            return builder.ToString();
        }

        /// <summary>
        /// Appends one trimmed fixed-scale fractional component using exactly the digits required by the stored scale.
        /// </summary>
        /// <param name="builder">Destination builder receiving the fractional digits.</param>
        /// <param name="fractionalPart">Scaled fractional component to serialize.</param>
        static void AppendFractionDigits(StringBuilder builder, long fractionalPart) {
            long trimmedFractionalPart = fractionalPart;
            int lastDigitIndex = 8;
            while (lastDigitIndex >= 0 && trimmedFractionalPart % 10L == 0L) {
                trimmedFractionalPart /= 10L;
                lastDigitIndex--;
            }

            long divisor = TraceFloatScale / 10L;
            for (int digitIndex = 0; digitIndex <= lastDigitIndex; digitIndex++) {
                long digit = fractionalPart / divisor;
                builder.Append((char)('0' + digit));
                fractionalPart -= digit * divisor;
                divisor /= 10L;
            }
        }

        /// <summary>
        /// Appends one side of a two-body solve snapshot using compact lane data.
        /// </summary>
        /// <param name="builder">Destination builder receiving the snapshot text.</param>
        /// <param name="sideName">Short side label being appended.</param>
        /// <param name="encodedBodyIndices">Encoded body references for the current side.</param>
        /// <param name="velocity">Wide velocities gathered for the current side.</param>
        static void AppendTwoBodySolveSide(StringBuilder builder, string sideName, Vector<int> encodedBodyIndices, BodyVelocityWide velocity) {
            builder.Append(" ");
            builder.Append(sideName);
            builder.Append("=<");
            for (int laneIndex = 0; laneIndex < Vector<int>.Count; laneIndex++) {
                if (laneIndex > 0) {
                    builder.Append(";");
                }

                builder.Append("body=");
                builder.Append(encodedBodyIndices[laneIndex]);
                builder.Append(",lyScaled=");
                builder.Append((int)(velocity.Linear.Y[laneIndex] * 1000000.0f));
                builder.Append(",azScaled=");
                builder.Append((int)(velocity.Angular.Z[laneIndex] * 1000000.0f));
            }

            builder.Append(">");
        }
    }
}
#endif
