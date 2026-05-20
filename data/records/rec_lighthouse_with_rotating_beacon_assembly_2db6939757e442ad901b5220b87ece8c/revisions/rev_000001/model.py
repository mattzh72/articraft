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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _polar(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def _add_radial_box(
    part,
    *,
    size: tuple[float, float, float],
    radius: float,
    angle: float,
    z: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=_polar(radius, angle, z), rpy=(0.0, 0.0, angle)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pier_lighthouse")

    masonry = model.material("masonry", rgba=(0.91, 0.92, 0.88, 1.0))
    stone_trim = model.material("stone_trim", rgba=(0.77, 0.78, 0.75, 1.0))
    gallery_metal = model.material("gallery_metal", rgba=(0.22, 0.24, 0.26, 1.0))
    lantern_frame = model.material("lantern_frame", rgba=(0.27, 0.31, 0.29, 1.0))
    roof_paint = model.material("roof_paint", rgba=(0.20, 0.37, 0.31, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.73, 0.87, 0.93, 0.28))
    beacon_lens = model.material("beacon_lens", rgba=(0.95, 0.86, 0.48, 0.52))
    bright_metal = model.material("bright_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    brass = model.material("brass", rgba=(0.71, 0.56, 0.23, 1.0))
    door_paint = model.material("door_paint", rgba=(0.46, 0.14, 0.10, 1.0))

    lighthouse_body = model.part("lighthouse_body")
    lighthouse_body.inertial = Inertial.from_geometry(
        Box((2.10, 2.10, 3.38)),
        mass=850.0,
        origin=Origin(xyz=(0.0, 0.0, 1.69)),
    )

    base_profile = [
        (0.0, 0.0),
        (1.05, 0.0),
        (1.05, 0.18),
        (0.97, 0.30),
        (0.90, 0.95),
        (0.84, 1.52),
        (0.79, 1.82),
        (0.88, 1.94),
        (0.94, 2.02),
        (0.0, 2.02),
    ]
    lighthouse_body.visual(
        _save_mesh("lighthouse_base_body", LatheGeometry(base_profile, segments=96)),
        material=masonry,
        name="base_body",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.94, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=stone_trim,
        name="belt_course",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.98, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.06)),
        material=stone_trim,
        name="gallery_deck",
    )
    for angle in (0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi, 5.0 * math.pi / 4.0, 3.0 * math.pi / 2.0, 7.0 * math.pi / 4.0):
        _add_radial_box(
            lighthouse_body,
            size=(0.18, 0.10, 0.20),
            radius=0.73,
            angle=angle,
            z=1.91,
            material=stone_trim,
        )

    lighthouse_body.visual(
        _save_mesh(
            "gallery_curb",
            TorusGeometry(radius=0.88, tube=0.03, radial_segments=16, tubular_segments=56).translate(0.0, 0.0, 2.13),
        ),
        material=gallery_metal,
        name="gallery_curb",
    )
    lighthouse_body.visual(
        _save_mesh(
            "gallery_mid_rail",
            TorusGeometry(radius=0.88, tube=0.016, radial_segments=16, tubular_segments=56).translate(0.0, 0.0, 2.31),
        ),
        material=gallery_metal,
        name="gallery_mid_rail",
    )
    lighthouse_body.visual(
        _save_mesh(
            "gallery_top_rail",
            TorusGeometry(radius=0.88, tube=0.02, radial_segments=16, tubular_segments=56).translate(0.0, 0.0, 2.44),
        ),
        material=gallery_metal,
        name="gallery_top_rail",
    )
    for index in range(16):
        angle = math.tau * index / 16.0
        lighthouse_body.visual(
            Cylinder(radius=0.012, length=0.26),
            origin=Origin(xyz=_polar(0.88, angle, 2.29)),
            material=gallery_metal,
        )

    lighthouse_body.visual(
        Cylinder(radius=0.50, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.17)),
        material=stone_trim,
        name="lantern_floor",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.15, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 2.295)),
        material=gallery_metal,
        name="spindle_pedestal",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.028, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 2.495)),
        material=bright_metal,
        name="spindle_shaft",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.08, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 2.355)),
        material=bright_metal,
        name="spindle_lower_retainer",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.08, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 2.755)),
        material=bright_metal,
        name="spindle_upper_retainer",
    )
    lighthouse_body.visual(
        _save_mesh(
            "lantern_sill",
            TorusGeometry(radius=0.42, tube=0.03, radial_segments=16, tubular_segments=56).translate(0.0, 0.0, 2.22),
        ),
        material=lantern_frame,
        name="lantern_sill",
    )
    lighthouse_body.visual(
        _save_mesh(
            "lantern_header",
            TorusGeometry(radius=0.42, tube=0.03, radial_segments=16, tubular_segments=56).translate(0.0, 0.0, 2.79),
        ),
        material=lantern_frame,
        name="lantern_header",
    )
    lighthouse_body.visual(
        _save_mesh(
            "roof_eave",
            TorusGeometry(radius=0.50, tube=0.035, radial_segments=16, tubular_segments=56).translate(0.0, 0.0, 2.84),
        ),
        material=roof_paint,
        name="roof_eave",
    )

    mullion_angles = (-2.83, -2.20, -1.57, -0.94, -0.23, 0.23, 0.94, 1.57, 2.20, 2.83)
    for index, angle in enumerate(mullion_angles):
        _add_radial_box(
            lighthouse_body,
            size=(0.045, 0.028, 0.56),
            radius=0.425,
            angle=angle,
            z=2.50,
            material=lantern_frame,
            name=f"mullion_{index}",
        )

    pane_angles = (-2.515, -1.885, -1.255, -0.585, 0.585, 1.255, 1.885, 2.515)
    for index, angle in enumerate(pane_angles):
        _add_radial_box(
            lighthouse_body,
            size=(0.008, 0.195, 0.55),
            radius=0.410,
            angle=angle,
            z=2.495,
            material=lantern_glass,
            name=f"pane_{index}",
        )

    lighthouse_body.visual(
        Box((0.040, 0.200, 0.530)),
        origin=Origin(xyz=(0.420, 0.0, 2.515)),
        material=lantern_frame,
        name="door_frame",
    )

    roof_profile = [
        (0.0, 0.0),
        (0.56, 0.0),
        (0.52, 0.03),
        (0.44, 0.12),
        (0.33, 0.24),
        (0.22, 0.34),
        (0.12, 0.42),
        (0.04, 0.48),
        (0.0, 0.50),
    ]
    lighthouse_body.visual(
        _save_mesh("lantern_roof", LatheGeometry(roof_profile, segments=72)),
        origin=Origin(xyz=(0.0, 0.0, 2.80)),
        material=roof_paint,
        name="lantern_roof",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.055, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 3.295)),
        material=gallery_metal,
        name="roof_vent",
    )
    lighthouse_body.visual(
        Sphere(radius=0.05),
        origin=Origin(xyz=(0.0, 0.0, 3.38)),
        material=gallery_metal,
        name="finial",
    )

    beacon_drum = model.part("beacon_drum")
    beacon_drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.37),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
    )
    beacon_outer = [
        (0.12, 0.0),
        (0.10, 0.025),
        (0.055, 0.045),
        (0.055, 0.085),
        (0.22, 0.105),
        (0.22, 0.265),
        (0.055, 0.285),
        (0.055, 0.325),
        (0.10, 0.345),
        (0.12, 0.37),
    ]
    beacon_inner = [
        (0.038, 0.0),
        (0.038, 0.075),
        (0.18, 0.115),
        (0.18, 0.255),
        (0.038, 0.295),
        (0.038, 0.37),
    ]
    beacon_drum.visual(
        _save_mesh(
            "beacon_drum_shell",
            LatheGeometry.from_shell_profiles(
                beacon_outer,
                beacon_inner,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=beacon_lens,
        name="beacon_drum",
    )
    beacon_drum.visual(
        _save_mesh(
            "beacon_hub_sleeve",
            LatheGeometry.from_shell_profiles(
                [(0.052, 0.02), (0.052, 0.35)],
                [(0.033, 0.02), (0.033, 0.35)],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=bright_metal,
        name="hub_sleeve",
    )
    beacon_drum.visual(
        Box((0.13, 0.024, 0.05)),
        origin=Origin(xyz=(0.115, 0.0, 0.185)),
        material=bright_metal,
        name="reflector_strut",
    )
    beacon_drum.visual(
        Box((0.13, 0.024, 0.05)),
        origin=Origin(xyz=(-0.115, 0.0, 0.185)),
        material=gallery_metal,
        name="balance_strut",
    )
    beacon_drum.visual(
        Box((0.07, 0.13, 0.13)),
        origin=Origin(xyz=(0.125, 0.0, 0.185)),
        material=bright_metal,
        name="reflector_body",
    )
    beacon_drum.visual(
        Cylinder(radius=0.028, length=0.10),
        origin=Origin(xyz=(0.145, 0.0, 0.185), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lamp_core",
    )
    beacon_drum.visual(
        Box((0.06, 0.06, 0.09)),
        origin=Origin(xyz=(-0.145, 0.0, 0.185)),
        material=gallery_metal,
        name="counterweight",
    )

    maintenance_door = model.part("maintenance_door")
    maintenance_door.inertial = Inertial.from_geometry(
        Box((0.03, 0.18, 0.38)),
        mass=5.0,
        origin=Origin(xyz=(0.011, -0.085, 0.185)),
    )
    maintenance_door.visual(
        Box((0.022, 0.162, 0.35)),
        origin=Origin(xyz=(0.011, -0.081, 0.185)),
        material=door_paint,
        name="door_panel",
    )
    maintenance_door.visual(
        Box((0.008, 0.126, 0.27)),
        origin=Origin(xyz=(0.021, -0.081, 0.185)),
        material=lantern_frame,
        name="door_inner_frame",
    )
    maintenance_door.visual(
        Cylinder(radius=0.045, length=0.006),
        origin=Origin(xyz=(0.018, -0.081, 0.235), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_metal,
        name="door_portlight",
    )
    maintenance_door.visual(
        Box((0.008, 0.018, 0.070)),
        origin=Origin(xyz=(0.024, -0.148, 0.185)),
        material=bright_metal,
        name="door_latch",
    )

    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=lighthouse_body,
        child=beacon_drum,
        origin=Origin(xyz=(0.0, 0.0, 2.37)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=lighthouse_body,
        child=maintenance_door,
        origin=Origin(xyz=(0.44, 0.085, 2.39)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lighthouse_body = object_model.get_part("lighthouse_body")
    beacon_drum = object_model.get_part("beacon_drum")
    maintenance_door = object_model.get_part("maintenance_door")
    beacon_spin = object_model.get_articulation("beacon_spin")
    door_hinge = object_model.get_articulation("door_hinge")

    beacon_visual = beacon_drum.get_visual("beacon_drum")
    spindle_shaft = lighthouse_body.get_visual("spindle_shaft")
    lower_retainer = lighthouse_body.get_visual("spindle_lower_retainer")
    upper_retainer = lighthouse_body.get_visual("spindle_upper_retainer")
    lantern_sill = lighthouse_body.get_visual("lantern_sill")
    door_panel = maintenance_door.get_visual("door_panel")
    door_frame = lighthouse_body.get_visual("door_frame")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
        "beacon spins as a vertical continuous articulation",
        beacon_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(beacon_spin.axis) == (0.0, 0.0, 1.0)
        and beacon_spin.motion_limits is not None
        and beacon_spin.motion_limits.lower is None
        and beacon_spin.motion_limits.upper is None,
        details=f"type={beacon_spin.articulation_type}, axis={beacon_spin.axis}, limits={beacon_spin.motion_limits}",
    )
    ctx.check(
        "maintenance door uses a vertical side hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )

    ctx.expect_overlap(
        beacon_drum,
        lighthouse_body,
        axes="xy",
        elem_a=beacon_visual,
        elem_b=spindle_shaft,
        min_overlap=0.05,
        name="beacon drum stays centered on spindle",
    )
    ctx.expect_within(
        beacon_drum,
        lighthouse_body,
        axes="xy",
        inner_elem=beacon_visual,
        outer_elem=lantern_sill,
        margin=0.0,
        name="beacon drum stays inside lantern housing",
    )
    ctx.expect_gap(
        beacon_drum,
        lighthouse_body,
        axis="z",
        positive_elem=beacon_visual,
        negative_elem=lower_retainer,
        max_penetration=0.001,
        max_gap=0.004,
        name="beacon drum seats on lower retainer",
    )
    ctx.expect_gap(
        lighthouse_body,
        beacon_drum,
        axis="z",
        positive_elem=upper_retainer,
        negative_elem=beacon_visual,
        max_penetration=0.001,
        max_gap=0.004,
        name="upper retainer clips beacon drum in place",
    )

    closed_door_aabb = ctx.part_element_world_aabb(maintenance_door, elem=door_panel)
    if closed_door_aabb is None:
        ctx.fail("maintenance door closed aabb available", "door panel AABB was not available in the rest pose")
    else:
        closed_center_x = 0.5 * (closed_door_aabb[0][0] + closed_door_aabb[1][0])
        closed_center_y = 0.5 * (closed_door_aabb[0][1] + closed_door_aabb[1][1])
        ctx.check(
            "maintenance door sits in the lantern wall bay when closed",
            0.44 <= closed_center_x <= 0.47
            and abs(closed_center_y) <= 0.02
            and closed_door_aabb[0][1] < -0.05
            and closed_door_aabb[1][1] > 0.05,
            details=f"closed={closed_door_aabb}",
        )
        ctx.expect_contact(
            maintenance_door,
            lighthouse_body,
            elem_a=door_panel,
            elem_b=door_frame,
            name="maintenance door closes against the lantern frame",
        )
        with ctx.pose({door_hinge: math.radians(72.0)}):
            open_door_aabb = ctx.part_element_world_aabb(maintenance_door, elem=door_panel)
            if open_door_aabb is None:
                ctx.fail("maintenance door open aabb available", "door panel AABB was not available in the open pose")
            else:
                open_center_x = 0.5 * (open_door_aabb[0][0] + open_door_aabb[1][0])
                open_center_y = 0.5 * (open_door_aabb[0][1] + open_door_aabb[1][1])
                swings_outward = (
                    math.hypot(open_center_x, open_center_y)
                    > math.hypot(closed_center_x, closed_center_y) + 0.05
                    and abs(open_center_y - closed_center_y) > 0.05
                )
                ctx.check(
                    "maintenance door swings outward from the lantern wall",
                    swings_outward,
                    details=f"closed={closed_door_aabb}, open={open_door_aabb}",
                )
            ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with maintenance door open")

    with ctx.pose({beacon_spin: math.pi / 2.0}):
        ctx.expect_gap(
            beacon_drum,
            lighthouse_body,
            axis="z",
            positive_elem=beacon_visual,
            negative_elem=lower_retainer,
            max_penetration=0.001,
            max_gap=0.004,
            name="beacon drum remains seated while rotating",
        )
        ctx.expect_gap(
            lighthouse_body,
            beacon_drum,
            axis="z",
            positive_elem=upper_retainer,
            negative_elem=beacon_visual,
            max_penetration=0.001,
            max_gap=0.004,
            name="upper retainer still captures beacon while rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
