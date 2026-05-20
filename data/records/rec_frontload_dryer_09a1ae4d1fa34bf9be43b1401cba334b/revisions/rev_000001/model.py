from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    mesh_rev = "v5"

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, f"{name}_{mesh_rev}")

    def _annulus_mesh(name: str, *, outer_diameter: float, inner_diameter: float, depth: float):
        outer = rounded_rect_profile(outer_diameter, outer_diameter, outer_diameter * 0.5, corner_segments=18)
        inner = superellipse_profile(inner_diameter, inner_diameter, exponent=2.0, segments=72)
        return _mesh(
            name,
            ExtrudeWithHolesGeometry(outer, [inner], height=depth, center=True).rotate_x(pi / 2.0),
        )

    def _annulus_z_mesh(name: str, *, outer_diameter: float, inner_diameter: float, depth: float):
        outer = rounded_rect_profile(outer_diameter, outer_diameter, outer_diameter * 0.5, corner_segments=18)
        inner = superellipse_profile(inner_diameter, inner_diameter, exponent=2.0, segments=72)
        return _mesh(name, ExtrudeWithHolesGeometry(outer, [inner], height=depth, center=True))

    model = ArticulatedObject(name="commercial_laundromat_dryer")

    cabinet_width = 0.96
    cabinet_depth = 1.02
    cabinet_height = 1.78
    panel_thickness = 0.022
    opening_center_z = cabinet_height * 0.5
    front_panel_y = cabinet_depth * 0.5 - panel_thickness * 0.5
    rear_panel_y = -front_panel_y
    opening_diameter = 0.62
    seal_outer_diameter = 0.70
    seal_inner_diameter = 0.60
    drum_radius = 0.34
    drum_depth = 0.66
    drum_center_y = -0.02
    door_outer_diameter = 0.78
    door_inner_diameter = 0.50
    door_thickness = 0.07
    door_center_offset_x = door_outer_diameter * 0.5 + 0.015
    hinge_axis_x = -door_center_offset_x
    hinge_axis_y = cabinet_depth * 0.5 + 0.035
    hinge_half_spacing = 0.22

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.61, 0.63, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    black_panel = model.material("black_panel", rgba=(0.12, 0.13, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("glass", rgba=(0.56, 0.67, 0.76, 0.35))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=245.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    cabinet.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width * 0.5 + panel_thickness * 0.5, 0.0, cabinet_height * 0.5)),
        material=stainless,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width * 0.5 - panel_thickness * 0.5, 0.0, cabinet_height * 0.5)),
        material=stainless,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, panel_thickness * 0.5)),
        material=dark_steel,
        name="base_pan",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - panel_thickness * 0.5)),
        material=stainless,
        name="top_panel",
    )
    cabinet.visual(
        Box((cabinet_width, panel_thickness, cabinet_height)),
        origin=Origin(xyz=(0.0, rear_panel_y, cabinet_height * 0.5)),
        material=stainless,
        name="rear_panel",
    )
    cabinet.visual(
        _mesh(
            "dryer_front_bezel",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(cabinet_width, cabinet_height, 0.026, corner_segments=10),
                [superellipse_profile(opening_diameter, opening_diameter, exponent=2.0, segments=80)],
                height=panel_thickness,
                center=True,
            ).rotate_x(pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, front_panel_y, cabinet_height * 0.5)),
        material=stainless,
        name="front_bezel",
    )
    cabinet.visual(
        _annulus_mesh(
            "dryer_front_seal_rim",
            outer_diameter=seal_outer_diameter,
            inner_diameter=seal_inner_diameter,
            depth=0.072,
        ),
        origin=Origin(xyz=(0.0, 0.474, opening_center_z)),
        material=brushed_steel,
        name="seal_rim",
    )
    cabinet.visual(
        Box((0.40, 0.034, 0.10)),
        origin=Origin(xyz=(0.18, 0.527, 1.58)),
        material=black_panel,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.18, 0.010, 0.045)),
        origin=Origin(xyz=(0.12, 0.549, 1.59)),
        material=glass,
        name="display_window",
    )
    cabinet.visual(
        Box((0.82, 0.018, 0.12)),
        origin=Origin(xyz=(0.0, 0.519, 0.14)),
        material=dark_steel,
        name="kick_plate",
    )
    cabinet.visual(
        Cylinder(radius=0.050, length=0.118),
        origin=Origin(xyz=(0.0, -0.452, opening_center_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing",
    )
    for hinge_name, hinge_z, bracket_center_z in (
        ("top", opening_center_z + hinge_half_spacing, opening_center_z + hinge_half_spacing + 0.110),
        ("bottom", opening_center_z - hinge_half_spacing, opening_center_z - hinge_half_spacing - 0.110),
    ):
        cabinet.visual(
            Box((0.060, 0.032, 0.110)),
            origin=Origin(xyz=(hinge_axis_x - 0.046, 0.526, bracket_center_z)),
            material=dark_steel,
            name=f"{hinge_name}_hinge_bracket",
        )
        cabinet.visual(
            Box((0.036, 0.016, 0.014)),
            origin=Origin(
                xyz=(
                    hinge_axis_x - 0.018,
                    hinge_axis_y - 0.024,
                    hinge_z + 0.062 if hinge_name == "top" else hinge_z - 0.062,
                )
            ),
            material=dark_steel,
            name=f"{hinge_name}_hinge_cap",
        )
        cabinet.visual(
            Cylinder(radius=0.016, length=0.110),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_z)),
            material=dark_steel,
            name=f"{hinge_name}_hinge_sleeve",
        )

    drum = model.part("drum")
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_radius, length=drum_depth),
        mass=82.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    drum.visual(
        _mesh(
            "dryer_drum_shell",
            CylinderGeometry(radius=drum_radius, height=drum_depth, radial_segments=80, closed=False).rotate_x(pi / 2.0),
        ),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        _annulus_mesh(
            "dryer_drum_front_hoop",
            outer_diameter=0.688,
            inner_diameter=0.590,
            depth=0.026,
        ),
        origin=Origin(xyz=(0.0, drum_depth * 0.5 - 0.013, 0.0)),
        material=brushed_steel,
        name="front_hoop",
    )
    drum.visual(
        _annulus_mesh(
            "dryer_drum_rear_hoop",
            outer_diameter=0.686,
            inner_diameter=0.586,
            depth=0.026,
        ),
        origin=Origin(xyz=(0.0, -drum_depth * 0.5 + 0.013, 0.0)),
        material=brushed_steel,
        name="rear_hoop",
    )
    drum.visual(
        Cylinder(radius=0.344, length=0.014),
        origin=Origin(xyz=(0.0, -drum_depth * 0.5 + 0.007, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="rear_plate",
    )
    drum.visual(
        Cylinder(radius=0.028, length=0.090),
        origin=Origin(xyz=(0.0, -0.360, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_outer_diameter, door_thickness, door_outer_diameter)),
        mass=18.0,
        origin=Origin(xyz=(door_center_offset_x, 0.0, 0.0)),
    )
    door.visual(
        _annulus_mesh(
            "dryer_door_outer_ring",
            outer_diameter=door_outer_diameter,
            inner_diameter=door_inner_diameter,
            depth=door_thickness,
        ),
        origin=Origin(xyz=(door_center_offset_x, 0.0, 0.0)),
        material=stainless,
        name="door_ring",
    )
    door.visual(
        _annulus_mesh(
            "dryer_door_inner_clamp",
            outer_diameter=0.58,
            inner_diameter=0.48,
            depth=0.020,
        ),
        origin=Origin(xyz=(door_center_offset_x, -0.018, 0.0)),
        material=brushed_steel,
        name="inner_clamp",
    )
    door.visual(
        Cylinder(radius=0.244, length=0.018),
        origin=Origin(xyz=(door_center_offset_x, -0.008, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        _mesh("dryer_door_gasket", TorusGeometry(radius=0.305, tube=0.016).rotate_x(pi / 2.0)),
        origin=Origin(xyz=(door_center_offset_x, -0.019, 0.0)),
        material=rubber,
        name="door_gasket",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(0.620, 0.026, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_stem",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.180),
        origin=Origin(xyz=(0.620, 0.064, 0.0)),
        material=dark_steel,
        name="handle_grip",
    )
    door.visual(
        Box((0.055, 0.032, 0.110)),
        origin=Origin(xyz=(0.593, 0.018, 0.0)),
        material=dark_steel,
        name="handle_mount",
    )
    door.visual(
        Box((0.160, 0.028, 0.620)),
        origin=Origin(xyz=(0.098, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_spine",
    )
    for hinge_name, hinge_local_z in (("top", hinge_half_spacing), ("bottom", -hinge_half_spacing)):
        door.visual(
            _annulus_z_mesh(
                f"dryer_{hinge_name}_door_knuckle",
                outer_diameter=0.036,
                inner_diameter=0.026,
                depth=0.108,
            ),
            origin=Origin(xyz=(0.0, 0.0, hinge_local_z)),
            material=dark_steel,
            name=f"{hinge_name}_door_knuckle",
        )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, drum_center_y, opening_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=12.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.8, lower=0.0, upper=1.92),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_spin = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")

    front_bezel = cabinet.get_visual("front_bezel")
    seal_rim = cabinet.get_visual("seal_rim")
    rear_bearing = cabinet.get_visual("rear_bearing")
    top_hinge_sleeve = cabinet.get_visual("top_hinge_sleeve")
    bottom_hinge_sleeve = cabinet.get_visual("bottom_hinge_sleeve")
    drum_shell = drum.get_visual("drum_shell")
    rear_axle = drum.get_visual("rear_axle")

    door_ring = door.get_visual("door_ring")
    door_gasket = door.get_visual("door_gasket")
    top_door_knuckle = door.get_visual("top_door_knuckle")
    bottom_door_knuckle = door.get_visual("bottom_door_knuckle")

    ctx.allow_overlap(
        drum,
        cabinet,
        elem_a=rear_axle,
        elem_b=rear_bearing,
        reason="The rear drum shaft is represented as a solid axle nested into a solid bearing sleeve proxy.",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=top_door_knuckle,
        elem_b=top_hinge_sleeve,
        reason="The upper dryer hinge is modeled with coaxial knuckle and sleeve solids so the heavy door stays physically mounted through its swing.",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=bottom_door_knuckle,
        elem_b=bottom_hinge_sleeve,
        reason="The lower dryer hinge is modeled with coaxial knuckle and sleeve solids so the heavy door stays physically mounted through its swing.",
    )

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
        "dryer_has_expected_parts",
        all(part_name in {part.name for part in object_model.parts} for part_name in ("cabinet", "drum", "door")),
        "Expected cabinet, drum, and door parts.",
    )
    ctx.check(
        "door_hinge_axis_vertical",
        tuple(round(value, 6) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        f"Door hinge axis should be vertical, got {door_hinge.axis}.",
    )
    ctx.check(
        "drum_axis_fore_aft",
        tuple(round(value, 6) for value in drum_spin.axis) == (0.0, 1.0, 0.0),
        f"Drum should spin on the fore-aft axis, got {drum_spin.axis}.",
    )

    door_limits = door_hinge.motion_limits
    drum_limits = drum_spin.motion_limits
    ctx.check(
        "door_opening_limits_realistic",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and 1.70 <= door_limits.upper <= 2.05,
        f"Door limits should describe a real side swing, got {door_limits}.",
    )
    ctx.check(
        "drum_continuous_limits_unbounded",
        drum_limits is not None and drum_limits.lower is None and drum_limits.upper is None,
        f"Continuous drum rotation should be unbounded, got {drum_limits}.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem=door_ring,
            negative_elem=front_bezel,
            name="door_ring_seats_flush_to_front_bezel",
        )
        ctx.expect_contact(
            door,
            cabinet,
            elem_a=door_gasket,
            elem_b=seal_rim,
            contact_tol=0.002,
            name="door_gasket_contacts_seal_rim",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.72,
            elem_a=door_ring,
            elem_b=front_bezel,
            name="door_covers_front_opening_projection",
        )
        ctx.expect_gap(
            cabinet,
            drum,
            axis="y",
            min_gap=0.08,
            max_gap=0.18,
            positive_elem=seal_rim,
            negative_elem=drum_shell,
            name="drum_sits_behind_front_seal_depth",
        )
        ctx.expect_within(
            drum,
            cabinet,
            axes="xz",
            margin=0.03,
            inner_elem=drum_shell,
            name="drum_shell_within_cabinet_width_height",
        )

    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_pose_no_unintended_overlap")
            ctx.fail_if_isolated_parts(name="door_closed_pose_no_floating")
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_pose_no_unintended_overlap")
            ctx.fail_if_isolated_parts(name="door_open_pose_no_floating")

    with ctx.pose({drum_spin: pi * 0.5}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drum_quarter_turn_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="drum_quarter_turn_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
