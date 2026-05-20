from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


SHOULDER_HEIGHT = 0.145
UPPER_LENGTH = 0.208
SLIDE_ENTRY_X = 0.113
RUNNER_TRAVEL = 0.055


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _support_pedestal_shape() -> cq.Workplane:
    base = _box((0.130, 0.105, 0.016), (0.0, 0.0, 0.008))
    column = _box((0.056, 0.046, 0.110), (-0.018, 0.0, 0.071))
    front_rib = _box((0.030, 0.038, 0.040), (0.008, 0.0, 0.036))
    return base.union(column).union(front_rib)


def _shoulder_yoke_shape() -> cq.Workplane:
    left_plate = _box((0.040, 0.008, 0.060), (-0.010, -0.016, SHOULDER_HEIGHT))
    right_plate = _box((0.040, 0.008, 0.060), (-0.010, 0.016, SHOULDER_HEIGHT))
    rear_bridge = _box((0.018, 0.040, 0.018), (-0.030, 0.0, SHOULDER_HEIGHT + 0.022))
    lower_web = _box((0.020, 0.040, 0.026), (-0.030, 0.0, SHOULDER_HEIGHT - 0.014))
    return left_plate.union(right_plate).union(rear_bridge).union(lower_web)


def _upper_pivot_lug_shape() -> cq.Workplane:
    lug = _box((0.022, 0.024, 0.038), (0.010, 0.0, 0.0))
    cheek = _box((0.020, 0.018, 0.026), (0.026, 0.0, 0.0))
    return lug.union(cheek)


def _upper_beam_shape() -> cq.Workplane:
    return _box((0.170, 0.022, 0.026), (0.096, 0.0, 0.0))


def _upper_yoke_shape() -> cq.Workplane:
    left_plate = _box((0.038, 0.008, 0.046), (0.189, -0.016, 0.0))
    right_plate = _box((0.038, 0.008, 0.046), (0.189, 0.016, 0.0))
    bridge = _box((0.022, 0.040, 0.018), (0.168, 0.0, 0.0))
    return left_plate.union(right_plate).union(bridge)


def _fore_pivot_lug_shape() -> cq.Workplane:
    lug = _box((0.020, 0.024, 0.032), (0.006, 0.0, 0.0))
    root = _box((0.020, 0.018, 0.022), (0.022, 0.0, 0.0))
    return lug.union(root)


def _fore_beam_shape() -> cq.Workplane:
    beam = _box((0.106, 0.020, 0.022), (0.053, 0.0, 0.0))
    transition = _box((0.020, 0.026, 0.024), (0.108, 0.0, 0.0))
    return beam.union(transition)


def _runner_sleeve_shape() -> cq.Workplane:
    base = _box((0.115, 0.036, 0.008), (0.167, 0.0, 0.010))
    left_keeper = _box((0.115, 0.006, 0.018), (0.167, -0.015, 0.015))
    right_keeper = _box((0.115, 0.006, 0.018), (0.167, 0.015, 0.015))
    nose = _box((0.014, 0.028, 0.014), (0.219, 0.0, 0.013))
    return base.union(left_keeper).union(right_keeper).union(nose)


def _runner_rail_shape() -> cq.Workplane:
    return _box((0.090, 0.022, 0.010), (0.045, 0.0, 0.005))


def _runner_pad_shape() -> cq.Workplane:
    stem = _box((0.016, 0.016, 0.010), (0.068, 0.0, 0.010))
    cap = _box((0.030, 0.028, 0.010), (0.080, 0.0, 0.018))
    nose = _box((0.016, 0.024, 0.008), (0.094, 0.0, 0.014))
    return stem.union(cap).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_link_arm_with_output_slide")

    model.material("support_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("link_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("runner_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("runner_cap", rgba=(0.20, 0.22, 0.24, 1.0))

    root_support = model.part("root_support")
    root_support.visual(
        mesh_from_cadquery(_support_pedestal_shape(), "support_pedestal"),
        material="support_steel",
        name="support_pedestal",
    )
    root_support.visual(
        mesh_from_cadquery(_shoulder_yoke_shape(), "shoulder_yoke"),
        material="support_steel",
        name="shoulder_yoke",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_pivot_lug_shape(), "upper_pivot_lug"),
        material="link_aluminum",
        name="upper_pivot_lug",
    )
    upper_link.visual(
        mesh_from_cadquery(_upper_beam_shape(), "upper_beam"),
        material="link_aluminum",
        name="upper_beam",
    )
    upper_link.visual(
        mesh_from_cadquery(_upper_yoke_shape(), "upper_yoke"),
        material="link_aluminum",
        name="upper_yoke",
    )

    forelink = model.part("forelink")
    forelink.visual(
        mesh_from_cadquery(_fore_pivot_lug_shape(), "fore_pivot_lug"),
        material="link_aluminum",
        name="fore_pivot_lug",
    )
    forelink.visual(
        mesh_from_cadquery(_fore_beam_shape(), "fore_beam"),
        material="link_aluminum",
        name="fore_beam",
    )
    forelink.visual(
        mesh_from_cadquery(_runner_sleeve_shape(), "runner_sleeve"),
        material="link_aluminum",
        name="runner_sleeve",
    )

    runner = model.part("runner")
    runner.visual(
        mesh_from_cadquery(_runner_rail_shape(), "runner_rail"),
        material="runner_dark",
        name="runner_rail",
    )
    runner.visual(
        mesh_from_cadquery(_runner_pad_shape(), "runner_pad"),
        material="runner_cap",
        name="runner_pad",
    )

    model.articulation(
        "support_to_upper",
        ArticulationType.REVOLUTE,
        parent=root_support,
        child=upper_link,
        origin=Origin(xyz=(0.010, 0.0, SHOULDER_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.15, effort=28.0, velocity=1.6),
    )
    model.articulation(
        "upper_to_forelink",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(UPPER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.05, upper=1.30, effort=18.0, velocity=1.8),
    )
    model.articulation(
        "forelink_to_runner",
        ArticulationType.PRISMATIC,
        parent=forelink,
        child=runner,
        origin=Origin(xyz=(SLIDE_ENTRY_X, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=RUNNER_TRAVEL, effort=12.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    root_support = object_model.get_part("root_support")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    runner = object_model.get_part("runner")

    shoulder = object_model.get_articulation("support_to_upper")
    elbow = object_model.get_articulation("upper_to_forelink")
    slide = object_model.get_articulation("forelink_to_runner")

    ctx.expect_overlap(
        upper_link,
        root_support,
        axes="yz",
        elem_a="upper_pivot_lug",
        elem_b="shoulder_yoke",
        min_overlap=0.014,
        name="upper pivot lug stays nested in the shoulder yoke footprint",
    )
    ctx.expect_overlap(
        forelink,
        upper_link,
        axes="yz",
        elem_a="fore_pivot_lug",
        elem_b="upper_yoke",
        min_overlap=0.014,
        name="forelink pivot lug stays nested in the upper yoke footprint",
    )
    ctx.expect_within(
        runner,
        forelink,
        axes="yz",
        inner_elem="runner_rail",
        outer_elem="runner_sleeve",
        margin=0.002,
        name="runner rail stays centered within the distal sleeve envelope",
    )
    ctx.expect_overlap(
        runner,
        forelink,
        axes="x",
        elem_a="runner_rail",
        elem_b="runner_sleeve",
        min_overlap=0.080,
        name="collapsed runner remains deeply inserted in the sleeve",
    )
    ctx.expect_contact(
        runner,
        forelink,
        elem_a="runner_rail",
        elem_b="runner_sleeve",
        contact_tol=0.0005,
        name="runner rail is seated on the distal guide at rest",
    )

    forelink_rest = ctx.part_world_position(forelink)
    with ctx.pose({shoulder: 0.80}):
        forelink_raised = ctx.part_world_position(forelink)

    runner_elbow_rest = None
    runner_elbow_raised = None
    with ctx.pose({shoulder: 0.35, elbow: 0.0}):
        runner_elbow_rest = ctx.part_world_position(runner)
    with ctx.pose({shoulder: 0.35, elbow: 0.95}):
        runner_elbow_raised = ctx.part_world_position(runner)

    runner_slide_rest = None
    runner_slide_extended = None
    with ctx.pose({shoulder: 0.0, elbow: 0.0, slide: 0.0}):
        ctx.expect_within(
            runner,
            forelink,
            axes="yz",
            inner_elem="runner_rail",
            outer_elem="runner_sleeve",
            margin=0.002,
            name="collapsed runner stays centered in the sleeve envelope",
        )
        runner_slide_rest = ctx.part_world_position(runner)
    with ctx.pose({shoulder: 0.0, elbow: 0.0, slide: RUNNER_TRAVEL}):
        ctx.expect_within(
            runner,
            forelink,
            axes="yz",
            inner_elem="runner_rail",
            outer_elem="runner_sleeve",
            margin=0.002,
            name="fully extended runner stays centered in the sleeve envelope",
        )
        ctx.expect_overlap(
            runner,
            forelink,
            axes="x",
            elem_a="runner_rail",
            elem_b="runner_sleeve",
            min_overlap=0.045,
            name="extended runner still retains insertion in the sleeve",
        )
        runner_slide_extended = ctx.part_world_position(runner)

    ctx.check(
        "shoulder positive motion raises the forelink",
        forelink_rest is not None
        and forelink_raised is not None
        and forelink_raised[2] > forelink_rest[2] + 0.10,
        details=f"rest={forelink_rest}, raised={forelink_raised}",
    )
    ctx.check(
        "elbow positive motion raises the distal assembly",
        runner_elbow_rest is not None
        and runner_elbow_raised is not None
        and runner_elbow_raised[2] > runner_elbow_rest[2] + 0.05,
        details=f"rest={runner_elbow_rest}, raised={runner_elbow_raised}",
    )
    ctx.check(
        "runner extends outward along +X",
        runner_slide_rest is not None
        and runner_slide_extended is not None
        and runner_slide_extended[0] > runner_slide_rest[0] + 0.02,
        details=f"rest={runner_slide_rest}, extended={runner_slide_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
