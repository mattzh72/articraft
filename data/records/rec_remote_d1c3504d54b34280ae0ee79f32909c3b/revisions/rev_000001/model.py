from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_tv_remote")

    shell_black = model.material("shell_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.15, 0.16, 0.17, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.63, 0.65, 0.67, 1.0))
    red = model.material("red", rgba=(0.72, 0.17, 0.16, 1.0))

    body = model.part("body")
    body_sections = [
        (-0.096, -0.0080, 0.0036, 0.028),
        (-0.074, -0.0083, 0.0058, 0.038),
        (-0.022, -0.0086, 0.0078, 0.045),
        (0.028, -0.0083, 0.0086, 0.044),
        (0.078, -0.0077, 0.0060, 0.034),
        (0.096, -0.0070, 0.0042, 0.022),
    ]
    body.visual(
        _mesh("remote_body_shell", superellipse_side_loft(body_sections, exponents=3.0, segments=56)),
        material=shell_black,
        name="body_shell",
    )
    body.visual(
        Box((0.034, 0.166, 0.0012)),
        origin=Origin(xyz=(0.0, 0.002, 0.0083)),
        material=graphite,
        name="face_panel",
    )
    body.visual(
        Box((0.013, 0.020, 0.0008)),
        origin=Origin(xyz=(0.0, 0.073, 0.0090)),
        material=soft_gray,
        name="ir_window",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0012),
        origin=Origin(xyz=(0.0, 0.086, 0.0092)),
        material=red,
        name="power_button",
    )
    body.visual(
        Box((0.010, 0.010, 0.0014)),
        origin=Origin(xyz=(-0.011, -0.032, 0.0092)),
        material=charcoal,
        name="back_button",
    )
    body.visual(
        Box((0.010, 0.010, 0.0014)),
        origin=Origin(xyz=(0.011, -0.032, 0.0092)),
        material=charcoal,
        name="home_button",
    )
    body.visual(
        Box((0.022, 0.008, 0.0014)),
        origin=Origin(xyz=(0.0, -0.050, 0.0092)),
        material=charcoal,
        name="play_button",
    )
    body.visual(
        Box((0.036, 0.136, 0.0008)),
        origin=Origin(xyz=(0.0, -0.015, -0.0084)),
        material=satin_black,
        name="door_track",
    )
    body.visual(
        Box((0.030, 0.087, 0.0014)),
        origin=Origin(xyz=(0.0, -0.021, -0.0076)),
        material=charcoal,
        name="battery_bay",
    )
    body.visual(
        Box((0.0022, 0.112, 0.0012)),
        origin=Origin(xyz=(-0.0162, -0.016, -0.0089)),
        material=satin_black,
        name="left_rear_rail",
    )
    body.visual(
        Box((0.0022, 0.112, 0.0012)),
        origin=Origin(xyz=(0.0162, -0.016, -0.0089)),
        material=satin_black,
        name="right_rear_rail",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.046, 0.192, 0.017)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
    )

    button_cluster = model.part("button_cluster")
    button_cluster.visual(
        Cylinder(radius=0.013, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0008)),
        material=charcoal,
        name="bearing_collar",
    )
    button_cluster.visual(
        Box((0.020, 0.008, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
        material=satin_black,
        name="dpad_horizontal",
    )
    button_cluster.visual(
        Box((0.008, 0.020, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
        material=satin_black,
        name="dpad_vertical",
    )
    button_cluster.visual(
        Cylinder(radius=0.006, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, 0.0027)),
        material=soft_gray,
        name="ok_button",
    )
    button_cluster.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.006),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.0030)),
    )

    ring = model.part("selector_ring")
    ring_geom = TorusGeometry(radius=0.0172, tube=0.0029, radial_segments=18, tubular_segments=56).scale(
        1.0, 1.0, 0.55
    )
    ring.visual(
        _mesh("selector_ring", ring_geom),
        material=soft_gray,
        name="selector_ring",
    )
    ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0205, length=0.0035),
        mass=0.008,
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        _mesh(
            "battery_door_panel",
            ExtrudeGeometry(rounded_rect_profile(0.0335, 0.093, 0.0045), 0.0016),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0008)),
        material=graphite,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.018, 0.008, 0.0008)),
        origin=Origin(xyz=(0.0, -0.038, -0.0013)),
        material=soft_gray,
        name="door_grip",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.034, 0.094, 0.0024)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, -0.0010)),
    )

    model.articulation(
        "body_to_button_cluster",
        ArticulationType.FIXED,
        parent=body,
        child=button_cluster,
        origin=Origin(xyz=(0.0, 0.014, 0.0089)),
    )
    model.articulation(
        "button_cluster_to_selector_ring",
        ArticulationType.CONTINUOUS,
        parent=button_cluster,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, 0.001571)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.020, -0.0092)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.055,
        ),
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

    body = object_model.get_part("body")
    button_cluster = object_model.get_part("button_cluster")
    ring = object_model.get_part("selector_ring")
    battery_door = object_model.get_part("battery_door")
    ring_joint = object_model.get_articulation("button_cluster_to_selector_ring")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.check(
        "selector ring uses a vertical continuous joint",
        ring_joint.axis == (0.0, 0.0, 1.0)
        and ring_joint.motion_limits is not None
        and ring_joint.motion_limits.lower is None
        and ring_joint.motion_limits.upper is None,
        details=f"axis={ring_joint.axis}, limits={ring_joint.motion_limits}",
    )
    ctx.check(
        "battery door slides toward the bottom end",
        door_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={door_joint.axis}",
    )

    ctx.expect_gap(
        button_cluster,
        body,
        axis="z",
        positive_elem="bearing_collar",
        negative_elem="face_panel",
        min_gap=0.0,
        max_gap=0.0006,
        name="button cluster mounts directly on the front face",
    )
    ctx.expect_gap(
        ring,
        body,
        axis="z",
        positive_elem="selector_ring",
        negative_elem="face_panel",
        min_gap=0.0,
        max_gap=0.0006,
        name="selector ring rides just above the front face",
    )
    ctx.expect_overlap(
        ring,
        body,
        axes="xy",
        elem_a="selector_ring",
        elem_b="face_panel",
        min_overlap=0.026,
        name="selector ring stays centered in the control zone",
    )

    closed_position = None
    open_position = None
    upper = door_joint.motion_limits.upper if door_joint.motion_limits is not None else 0.0

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_overlap(
            battery_door,
            body,
            axes="xy",
            elem_a="door_panel",
            elem_b="door_track",
            min_overlap=0.030,
            name="battery door covers the rear track when closed",
        )
        ctx.expect_gap(
            body,
            battery_door,
            axis="z",
            positive_elem="door_track",
            negative_elem="door_panel",
            min_gap=0.0001,
            max_gap=0.0010,
            name="battery door sits slightly proud of the rear shell",
        )
        closed_position = ctx.part_world_position(battery_door)

    with ctx.pose({door_joint: upper}):
        ctx.expect_overlap(
            battery_door,
            body,
            axes="xy",
            elem_a="door_panel",
            elem_b="door_track",
            min_overlap=0.028,
            name="battery door remains retained on the rear track when open",
        )
        open_position = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door translates downward when opened",
        closed_position is not None
        and open_position is not None
        and open_position[1] < closed_position[1] - 0.04,
        details=f"closed={closed_position}, open={open_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
