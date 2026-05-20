from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_trackpoint_keyboard")

    case_dark = model.material("case_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    deck_dark = model.material("deck_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    key_light = model.material("key_light", rgba=(0.82, 0.82, 0.80, 1.0))
    key_dark = model.material("key_dark", rgba=(0.36, 0.37, 0.39, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.60, 0.61, 0.63, 1.0))
    mount_dark = model.material("mount_dark", rgba=(0.12, 0.12, 0.12, 1.0))
    track_red = model.material("track_red", rgba=(0.74, 0.10, 0.10, 1.0))

    def _keycap_mesh(
        mesh_name: str,
        *,
        width: float,
        depth: float,
        height: float,
        top_shift_y: float,
    ):
        lower = rounded_rect_profile(
            width,
            depth,
            radius=min(width, depth) * 0.15,
            corner_segments=5,
        )
        middle = rounded_rect_profile(
            width * 0.95,
            depth * 0.92,
            radius=min(width, depth) * 0.13,
            corner_segments=5,
        )
        upper = rounded_rect_profile(
            width * 0.86,
            depth * 0.79,
            radius=min(width, depth) * 0.10,
            corner_segments=5,
        )
        geom = LoftGeometry(
            [
                [(x, y, 0.0) for x, y in lower],
                [(x, y + top_shift_y * 0.45, height * 0.55) for x, y in middle],
                [(x, y + top_shift_y, height) for x, y in upper],
            ],
            cap=True,
            closed=True,
        )
        return mesh_from_geometry(geom, mesh_name)

    def _wheel_mesh(mesh_name: str):
        geom = TorusGeometry(
            0.0072,
            0.0018,
            radial_segments=20,
            tubular_segments=48,
        )
        geom.merge(
            TorusGeometry(
                0.0074,
                0.0019,
                radial_segments=20,
                tubular_segments=48,
            )
        )
        geom.merge(
            TorusGeometry(
                0.0076,
                0.0017,
                radial_segments=20,
                tubular_segments=48,
            ).translate(0.0, 0.0, -0.0032)
        )
        geom.merge(
            TorusGeometry(
                0.0076,
                0.0017,
                radial_segments=20,
                tubular_segments=48,
            ).translate(0.0, 0.0, 0.0032)
        )
        return mesh_from_geometry(geom, mesh_name)

    case = model.part("case")
    case.visual(
        Box((0.278, 0.118, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=case_dark,
        name="bottom_shell",
    )
    case.visual(
        Box((0.278, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.054, 0.007)),
        material=case_dark,
        name="front_lip",
    )
    case.visual(
        Box((0.278, 0.008, 0.031)),
        origin=Origin(xyz=(0.0, 0.055, 0.0155)),
        material=case_dark,
        name="rear_wall",
    )
    case.visual(
        Box((0.008, 0.100, 0.025)),
        origin=Origin(xyz=(-0.135, 0.0, 0.0125)),
        material=case_dark,
        name="left_wall",
    )
    case.visual(
        Box((0.008, 0.100, 0.025)),
        origin=Origin(xyz=(0.135, 0.0, 0.0125)),
        material=case_dark,
        name="right_wall",
    )
    case.visual(
        Box((0.238, 0.090, 0.004)),
        origin=Origin(xyz=(0.0, -0.002, 0.010)),
        material=deck_dark,
        name="key_deck",
    )
    case.visual(
        Box((0.238, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.045, 0.013)),
        material=deck_dark,
        name="rear_control_shelf",
    )
    case.visual(
        Box((0.036, 0.028, 0.006)),
        origin=Origin(xyz=(0.108, 0.045, 0.009)),
        material=deck_dark,
        name="media_pod_base",
    )
    case.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(0.110, 0.033, 0.022)),
        material=case_dark,
        name="media_pod_front_cheek",
    )
    case.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(0.110, 0.057, 0.022)),
        material=case_dark,
        name="media_pod_rear_cheek",
    )
    case.visual(
        Box((0.006, 0.024, 0.018)),
        origin=Origin(xyz=(0.126, 0.045, 0.021)),
        material=case_dark,
        name="media_pod_outer_wall",
    )
    case.visual(
        Box((0.004, 0.018, 0.012)),
        origin=Origin(xyz=(0.094, 0.045, 0.018)),
        material=case_dark,
        name="media_pod_inner_boss",
    )
    case.inertial = Inertial.from_geometry(
        Box((0.278, 0.118, 0.031)),
        mass=1.85,
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
    )

    column_centers = [-0.0895, -0.0705, -0.0515, -0.0325, -0.0135, 0.0135, 0.0325, 0.0515, 0.0705, 0.0895]
    row_specs = (
        {"row": 0, "y": -0.028, "z": 0.0146, "shift": 0.0013, "offset": 0.0060},
        {"row": 1, "y": -0.009, "z": 0.0158, "shift": 0.0005, "offset": 0.0000},
        {"row": 2, "y": 0.010, "z": 0.0171, "shift": -0.0004, "offset": -0.0040},
        {"row": 3, "y": 0.029, "z": 0.0183, "shift": -0.0012, "offset": -0.0080},
    )
    key_meshes = {
        spec["row"]: _keycap_mesh(
            f"compact_keyboard_row_{spec['row']}_keycap",
            width=0.0162,
            depth=0.0162,
            height=0.0090 - spec["row"] * 0.0002,
            top_shift_y=spec["shift"],
        )
        for spec in row_specs
    }

    for spec in row_specs:
        row = spec["row"]
        for col, base_x in enumerate(column_centers):
            key = model.part(f"key_r{row}_c{col}")
            key.visual(
                key_meshes[row],
                material=key_dark if row == 3 or col in {0, 9} else key_light,
                name="keycap",
            )
            key.inertial = Inertial.from_geometry(
                Box((0.0162, 0.0162, 0.0090)),
                mass=0.014,
                origin=Origin(xyz=(0.0, 0.0, 0.0045)),
            )
            model.articulation(
                f"case_to_key_r{row}_c{col}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=key,
                origin=Origin(xyz=(base_x + spec["offset"], spec["y"], spec["z"])),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=1.2,
                    velocity=0.08,
                    lower=0.0,
                    upper=0.0022,
                ),
            )

    trackpoint_mount = model.part("trackpoint_mount")
    trackpoint_mount.visual(
        Cylinder(radius=0.0046, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
        material=mount_dark,
        name="mount_body",
    )
    trackpoint_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0046, length=0.0040),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
    )
    model.articulation(
        "case_to_trackpoint_mount",
        ArticulationType.REVOLUTE,
        parent=case,
        child=trackpoint_mount,
        origin=Origin(xyz=(0.0, -0.009, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=2.0,
            lower=-0.18,
            upper=0.18,
        ),
    )

    trackpoint_nub = model.part("trackpoint_nub")
    trackpoint_nub.visual(
        Cylinder(radius=0.0031, length=0.0080),
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=track_red,
        name="nub_stem",
    )
    trackpoint_nub.visual(
        Sphere(radius=0.0036),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=track_red,
        name="nub_tip",
    )
    trackpoint_nub.inertial = Inertial.from_geometry(
        Box((0.0072, 0.0072, 0.0145)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.0072)),
    )
    model.articulation(
        "trackpoint_mount_to_nub",
        ArticulationType.REVOLUTE,
        parent=trackpoint_mount,
        child=trackpoint_nub,
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.05,
            velocity=2.0,
            lower=-0.18,
            upper=0.18,
        ),
    )

    wheel_shaft = model.part("wheel_shaft")
    wheel_shaft.visual(
        Cylinder(radius=0.0025, length=0.0200),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="spindle",
    )
    wheel_shaft.visual(
        Cylinder(radius=0.0042, length=0.0020),
        origin=Origin(xyz=(0.0, -0.0090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="front_shoulder",
    )
    wheel_shaft.visual(
        Cylinder(radius=0.0048, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0092, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="rear_clip",
    )
    wheel_shaft.inertial = Inertial.from_geometry(
        Box((0.010, 0.020, 0.010)),
        mass=0.012,
        origin=Origin(),
    )
    model.articulation(
        "case_to_wheel_shaft",
        ArticulationType.FIXED,
        parent=case,
        child=wheel_shaft,
        origin=Origin(xyz=(0.110, 0.045, 0.024)),
    )

    media_wheel = model.part("media_wheel")
    media_wheel.visual(
        _wheel_mesh("compact_keyboard_media_wheel"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_dark,
        name="wheel_body",
    )
    media_wheel.inertial = Inertial.from_geometry(
        Box((0.022, 0.013, 0.022)),
        mass=0.028,
        origin=Origin(),
    )
    model.articulation(
        "wheel_shaft_to_media_wheel",
        ArticulationType.CONTINUOUS,
        parent=wheel_shaft,
        child=media_wheel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    home_left = object_model.get_part("key_r1_c4")
    front_center = object_model.get_part("key_r0_c4")
    top_center = object_model.get_part("key_r3_c4")
    trackpoint_mount = object_model.get_part("trackpoint_mount")
    trackpoint_nub = object_model.get_part("trackpoint_nub")
    wheel_shaft = object_model.get_part("wheel_shaft")
    media_wheel = object_model.get_part("media_wheel")

    key_joint = object_model.get_articulation("case_to_key_r1_c4")
    trackpoint_pitch = object_model.get_articulation("case_to_trackpoint_mount")
    trackpoint_roll = object_model.get_articulation("trackpoint_mount_to_nub")
    wheel_joint = object_model.get_articulation("wheel_shaft_to_media_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for row in range(4):
        for col in range(10):
            ctx.allow_isolated_part(
                object_model.get_part(f"key_r{row}_c{col}"),
                reason="Each key is an internally guided plunger with deliberate running clearance to the tray visuals.",
            )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "key_joint_is_vertical_prismatic",
        key_joint.articulation_type == ArticulationType.PRISMATIC and tuple(key_joint.axis) == (0.0, 0.0, -1.0),
        f"expected PRISMATIC key joint on -Z, got type={key_joint.articulation_type} axis={key_joint.axis}",
    )
    ctx.check(
        "trackpoint_joint_stack",
        trackpoint_pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(trackpoint_pitch.axis) == (1.0, 0.0, 0.0)
        and trackpoint_roll.articulation_type == ArticulationType.REVOLUTE
        and tuple(trackpoint_roll.axis) == (0.0, 1.0, 0.0),
        "trackpoint should tilt on nested X and Y revolute joints",
    )
    ctx.check(
        "wheel_joint_is_continuous",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        f"expected CONTINUOUS wheel joint on Y, got type={wheel_joint.articulation_type} axis={wheel_joint.axis}",
    )

    ctx.expect_gap(
        home_left,
        case,
        axis="z",
        min_gap=0.0020,
        max_gap=0.0040,
        negative_elem="key_deck",
        name="home_key_rest_clearance_to_deck",
    )
    with ctx.pose({key_joint: 0.0022}):
        ctx.expect_gap(
            home_left,
            case,
            axis="z",
            min_gap=0.0012,
            max_gap=0.0018,
            negative_elem="key_deck",
            name="home_key_pressed_travel_stays_short",
        )

    ctx.expect_origin_gap(
        top_center,
        front_center,
        axis="z",
        min_gap=0.0030,
        max_gap=0.0050,
        name="rows_step_up_toward_rear",
    )

    ctx.expect_contact(
        trackpoint_mount,
        case,
        elem_a="mount_body",
        elem_b="key_deck",
        name="trackpoint_mount_seats_on_deck",
    )
    ctx.expect_gap(
        trackpoint_nub,
        case,
        axis="z",
        positive_elem="nub_tip",
        min_gap=0.0090,
        max_gap=0.0140,
        negative_elem="key_deck",
        name="trackpoint_nub_protrudes_above_key_field",
    )
    with ctx.pose({trackpoint_pitch: 0.10, trackpoint_roll: -0.10}):
        ctx.expect_gap(
            trackpoint_nub,
            case,
            axis="z",
            positive_elem="nub_tip",
            min_gap=0.0090,
            max_gap=0.0140,
            negative_elem="key_deck",
            name="trackpoint_tilt_remains_proud_of_deck",
        )

    ctx.expect_contact(
        wheel_shaft,
        case,
        elem_a="front_shoulder",
        elem_b="media_pod_front_cheek",
        name="wheel_shaft_front_mount_contact",
    )
    ctx.expect_contact(
        wheel_shaft,
        case,
        elem_a="rear_clip",
        elem_b="media_pod_rear_cheek",
        name="wheel_shaft_rear_capture_contact",
    )
    ctx.expect_gap(
        media_wheel,
        wheel_shaft,
        axis="y",
        positive_elem="wheel_body",
        negative_elem="front_shoulder",
        min_gap=0.0020,
        max_gap=0.0045,
        name="media_wheel_front_capture_clearance",
    )
    ctx.expect_gap(
        wheel_shaft,
        media_wheel,
        axis="y",
        positive_elem="rear_clip",
        negative_elem="wheel_body",
        min_gap=0.0020,
        max_gap=0.0045,
        name="media_wheel_rear_capture_clearance",
    )
    ctx.expect_gap(
        media_wheel,
        case,
        axis="z",
        positive_elem="wheel_body",
        negative_elem="media_pod_base",
        min_gap=0.0020,
        max_gap=0.0035,
        name="media_wheel_above_pod_floor",
    )
    ctx.expect_gap(
        media_wheel,
        case,
        axis="x",
        negative_elem="media_pod_inner_boss",
        min_gap=0.0020,
        max_gap=0.0050,
        name="media_wheel_inner_side_clearance",
    )
    ctx.expect_gap(
        case,
        media_wheel,
        axis="x",
        positive_elem="media_pod_outer_wall",
        negative_elem="wheel_body",
        min_gap=0.0025,
        max_gap=0.0045,
        name="media_wheel_outer_side_clearance",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
