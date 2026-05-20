from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _body_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0425, 0.0000),
            (0.0450, 0.0022),
            (0.0450, 0.0108),
            (0.0438, 0.0140),
        ],
        [
            (0.0345, 0.0008),
            (0.0365, 0.0028),
            (0.0368, 0.0112),
            (0.0355, 0.0124),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _selector_ring_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0330, 0.0000),
            (0.0345, 0.0005),
            (0.0345, 0.0028),
            (0.0331, 0.0034),
        ],
        [
            (0.0245, 0.0006),
            (0.0240, 0.0010),
            (0.0240, 0.0025),
            (0.0232, 0.0030),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_home_puck_remote")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    ring_charcoal = model.material("ring_charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.75, 0.77, 0.79, 1.0))

    body = model.part("body")
    body.visual(
        _mesh("puck_body_shell", _body_shell_mesh()),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.039, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, 0.0127)),
        material=shell_white,
        name="top_deck",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0149)),
        material=accent_gray,
        name="center_pad",
    )
    body.visual(
        Box((0.060, 0.034, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0056)),
        material=accent_gray,
        name="battery_track_ceiling",
    )
    body.visual(
        Box((0.064, 0.004, 0.0054)),
        origin=Origin(xyz=(0.0, 0.016, 0.0087)),
        material=accent_gray,
        name="battery_track_left_rail",
    )
    body.visual(
        Box((0.064, 0.004, 0.0054)),
        origin=Origin(xyz=(0.0, -0.016, 0.0087)),
        material=accent_gray,
        name="battery_track_right_rail",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.015),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
    )

    selector_ring = model.part("selector_ring")
    selector_ring.visual(
        _mesh("puck_selector_ring", _selector_ring_mesh()),
        material=ring_charcoal,
        name="selector_ring_band",
    )
    selector_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0345, length=0.0034),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.060, 0.031, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0011)),
        material=accent_gray,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.040, 0.020, 0.0026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=accent_gray,
        name="door_tongue",
    )
    battery_door.visual(
        Box((0.014, 0.010, 0.0014)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0029)),
        material=accent_gray,
        name="door_grip",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.060, 0.031, 0.0050)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
    )

    model.articulation(
        "body_to_selector_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=0.016,
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
    selector_ring = object_model.get_part("selector_ring")
    battery_door = object_model.get_part("battery_door")
    ring_joint = object_model.get_articulation("body_to_selector_ring")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.check(
        "selector ring uses continuous vertical rotation",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(ring_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={ring_joint.articulation_type}, axis={ring_joint.axis}",
    )
    ctx.check(
        "battery door uses diametral prismatic slide",
        door_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(door_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )

    with ctx.pose({ring_joint: 0.0, door_joint: 0.0}):
        ctx.expect_contact(
            body,
            selector_ring,
            elem_a="top_deck",
            elem_b="selector_ring_band",
            contact_tol=1e-6,
            name="selector ring rides on the top deck bearing surface",
        )
        ctx.expect_overlap(
            selector_ring,
            body,
            axes="xy",
            elem_a="selector_ring_band",
            elem_b="top_deck",
            min_overlap=0.060,
            name="selector ring stays centered on the puck top",
        )
        ctx.expect_within(
            battery_door,
            body,
            axes="y",
            inner_elem="door_tongue",
            outer_elem="battery_track_ceiling",
            margin=0.0011,
            name="battery door tongue stays within the underside track",
        )
        ctx.expect_contact(
            body,
            battery_door,
            elem_a="battery_track_ceiling",
            elem_b="door_tongue",
            contact_tol=1e-6,
            name="battery door tongue stays in sliding contact with the track ceiling",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="x",
            elem_a="door_tongue",
            elem_b="battery_track_ceiling",
            min_overlap=0.038,
            name="closed battery door remains fully engaged in the track",
        )

    ring_rest = ctx.part_world_position(selector_ring)
    with ctx.pose({ring_joint: 1.8}):
        ring_turned = ctx.part_world_position(selector_ring)

    ctx.check(
        "selector ring turns about its center without translating",
        ring_rest is not None
        and ring_turned is not None
        and abs(ring_turned[0] - ring_rest[0]) <= 1e-6
        and abs(ring_turned[1] - ring_rest[1]) <= 1e-6
        and abs(ring_turned[2] - ring_rest[2]) <= 1e-6,
        details=f"rest={ring_rest}, turned={ring_turned}",
    )

    door_rest = ctx.part_world_position(battery_door)
    with ctx.pose({door_joint: 0.016}):
        ctx.expect_within(
            battery_door,
            body,
            axes="y",
            inner_elem="door_tongue",
            outer_elem="battery_track_ceiling",
            margin=0.0011,
            name="extended battery door stays guided between the rails",
        )
        ctx.expect_contact(
            body,
            battery_door,
            elem_a="battery_track_ceiling",
            elem_b="door_tongue",
            contact_tol=1e-6,
            name="extended battery door still rides in the track",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="x",
            elem_a="door_tongue",
            elem_b="battery_track_ceiling",
            min_overlap=0.030,
            name="extended battery door keeps retained insertion",
        )
        door_extended = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door slides outward along one disc diameter",
        door_rest is not None
        and door_extended is not None
        and door_extended[0] > door_rest[0] + 0.012
        and abs(door_extended[1] - door_rest[1]) <= 1e-6
        and abs(door_extended[2] - door_rest[2]) <= 1e-6,
        details=f"rest={door_rest}, extended={door_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
