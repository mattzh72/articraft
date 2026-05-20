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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _rr_section(width: float, depth: float, z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    radial_segments: int = 56,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _base_housing_mesh() -> MeshGeometry:
    return section_loft(
        [
            _rr_section(0.240, 0.210, 0.000, 0.034),
            _rr_section(0.226, 0.196, 0.060, 0.030),
            _rr_section(0.190, 0.158, 0.118, 0.024),
        ]
    )


def _jar_body_mesh() -> MeshGeometry:
    wall = _ring_band(
        outer_radius=0.078,
        inner_radius=0.072,
        height=0.242,
        z_center=0.139,
        radial_segments=64,
    )
    floor = CylinderGeometry(radius=0.072, height=0.006, radial_segments=64).translate(0.0, 0.0, 0.021)
    rim = _ring_band(
        outer_radius=0.082,
        inner_radius=0.0725,
        height=0.010,
        z_center=0.260,
        radial_segments=64,
    )
    return _merge([wall, floor, rim])


def _coupling_collar_mesh() -> MeshGeometry:
    collar = _ring_band(
        outer_radius=0.060,
        inner_radius=0.050,
        height=0.024,
        z_center=0.012,
        radial_segments=56,
    )
    collar.merge(
        CylinderGeometry(radius=0.010, height=0.006, radial_segments=20).translate(0.053, 0.0, 0.015)
    )
    collar.merge(
        CylinderGeometry(radius=0.010, height=0.006, radial_segments=20).translate(-0.053, 0.0, 0.015)
    )
    return collar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_bar_blender")

    base_black = model.material("base_black", rgba=(0.08, 0.08, 0.09, 1.0))
    deck_black = model.material("deck_black", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    lid_black = model.material("lid_black", rgba=(0.12, 0.12, 0.13, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        _mesh("base_housing", _base_housing_mesh()),
        material=base_black,
        name="base_housing",
    )
    base.visual(
        Box((0.164, 0.126, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=deck_black,
        name="top_deck",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=deck_black,
        name="drive_socket",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.110, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="speed_dial",
    )
    base.visual(
        Box((0.012, 0.036, 0.010)),
        origin=Origin(xyz=(0.121, 0.0, 0.052)),
        material=knob_black,
        name="speed_dial_grip",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.240, 0.210, 0.144)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
    )

    jar = model.part("jar")
    jar.visual(
        _mesh("jar_body", _jar_body_mesh()),
        material=stainless,
        name="jar_body",
    )
    jar.visual(
        _mesh("coupling_collar", _coupling_collar_mesh()),
        material=deck_black,
        name="coupling_collar",
    )
    jar.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(-0.024, -0.078, 0.264)),
        material=deck_black,
        name="hinge_bridge_left",
    )
    jar.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(0.024, -0.078, 0.264)),
        material=deck_black,
        name="hinge_bridge_right",
    )
    jar.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(-0.018, -0.078, 0.268), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deck_black,
        name="left_hinge_ear",
    )
    jar.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.018, -0.078, 0.268), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deck_black,
        name="right_hinge_ear",
    )
    jar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.270),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.007, 0.0, 0.074)),
        material=lid_black,
        name="upper_mount",
    )
    handle.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.007, 0.0, -0.074)),
        material=lid_black,
        name="lower_mount",
    )
    handle.visual(
        _mesh(
            "jar_handle_grip",
            tube_from_spline_points(
                [
                    (0.014, 0.0, 0.074),
                    (0.036, 0.0, 0.070),
                    (0.055, 0.0, 0.034),
                    (0.060, 0.0, 0.000),
                    (0.055, 0.0, -0.034),
                    (0.036, 0.0, -0.070),
                    (0.014, 0.0, -0.074),
                ],
                radius=0.006,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=lid_black,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.060, 0.024, 0.170)),
        mass=0.18,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_black,
        name="lid_barrel",
    )
    lid.visual(
        Cylinder(radius=0.076, length=0.008),
        origin=Origin(xyz=(0.0, 0.078, 0.004)),
        material=lid_black,
        name="lid_panel",
    )
    lid.visual(
        Box((0.030, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.148, 0.009)),
        material=lid_black,
        name="lid_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.078, length=0.014),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.078, 0.004)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=blade_steel,
        name="blade_shaft",
    )
    blade_assembly.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=blade_steel,
        name="blade_hub",
    )
    blade_assembly.visual(
        Box((0.056, 0.012, 0.003)),
        origin=Origin(xyz=(0.020, 0.0, 0.014), rpy=(0.0, 0.20, 0.0)),
        material=blade_steel,
        name="blade_primary",
    )
    blade_assembly.visual(
        Box((0.046, 0.010, 0.003)),
        origin=Origin(xyz=(-0.017, 0.0, 0.011), rpy=(0.0, -0.18, 0.0)),
        material=blade_steel,
        name="blade_secondary",
    )
    blade_assembly.visual(
        Box((0.010, 0.052, 0.003)),
        origin=Origin(xyz=(0.0, 0.018, 0.013), rpy=(0.20, 0.0, 0.0)),
        material=blade_steel,
        name="blade_cross_a",
    )
    blade_assembly.visual(
        Box((0.010, 0.044, 0.003)),
        origin=Origin(xyz=(0.0, -0.016, 0.010), rpy=(-0.18, 0.0, 0.0)),
        material=blade_steel,
        name="blade_cross_b",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.040),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "base_to_jar_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(30.0),
        ),
    )
    model.articulation(
        "jar_to_handle",
        ArticulationType.FIXED,
        parent=jar,
        child=handle,
        origin=Origin(xyz=(0.078, 0.0, 0.145)),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, -0.078, 0.268)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=1.9,
        ),
    )
    model.articulation(
        "jar_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=35.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    handle = object_model.get_part("handle")
    lid = object_model.get_part("lid")
    blade_assembly = object_model.get_part("blade_assembly")

    jar_lock = object_model.get_articulation("base_to_jar_lock")
    lid_hinge = object_model.get_articulation("jar_to_lid")
    blade_spin = object_model.get_articulation("jar_to_blade_assembly")

    def _axis_is(joint, expected: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(joint.axis, expected))

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.check(
        "jar lock twists around vertical axis",
        jar_lock.articulation_type == ArticulationType.REVOLUTE and _axis_is(jar_lock, (0.0, 0.0, 1.0)),
        details=f"type={jar_lock.articulation_type}, axis={jar_lock.axis}",
    )
    ctx.check(
        "lid hinge opens about lateral edge axis",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE and _axis_is(lid_hinge, (1.0, 0.0, 0.0)),
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    ctx.check(
        "blade assembly spins continuously about vertical axis",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS and _axis_is(blade_spin, (0.0, 0.0, 1.0)),
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            jar,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="jar_body",
            min_gap=0.001,
            max_gap=0.012,
            name="closed lid sits just above the jar rim",
        )
        ctx.expect_overlap(
            lid,
            jar,
            axes="xy",
            elem_a="lid_panel",
            elem_b="jar_body",
            min_overlap=0.130,
            name="closed lid covers the jar opening footprint",
        )

    ctx.expect_contact(
        handle,
        jar,
        elem_a="upper_mount",
        elem_b="jar_body",
        contact_tol=0.001,
        name="upper handle mount contacts the jar body",
    )
    ctx.expect_contact(
        handle,
        jar,
        elem_a="lower_mount",
        elem_b="jar_body",
        contact_tol=0.001,
        name="lower handle mount contacts the jar body",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        elem_a="coupling_collar",
        elem_b="drive_socket",
        min_overlap=0.085,
        name="jar collar remains centered over the drive socket",
    )

    closed_lid_center = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_panel"))
    open_lid_center = None
    with ctx.pose({lid_hinge: 1.4}):
        open_lid_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_panel"))
    ctx.check(
        "lid opens upward from the jar top",
        closed_lid_center is not None
        and open_lid_center is not None
        and open_lid_center[2] > closed_lid_center[2] + 0.045,
        details=f"closed_center={closed_lid_center}, open_center={open_lid_center}",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    turned_handle_pos = None
    with ctx.pose({jar_lock: math.radians(30.0)}):
        turned_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "jar bayonet lock twists the jar around the base socket",
        rest_handle_pos is not None
        and turned_handle_pos is not None
        and abs(turned_handle_pos[1] - rest_handle_pos[1]) > 0.030,
        details=f"rest={rest_handle_pos}, turned={turned_handle_pos}",
    )

    blade_center_rest = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_primary"))
    blade_center_spun = None
    with ctx.pose({blade_spin: math.pi / 2.0}):
        blade_center_spun = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_primary"))
    ctx.check(
        "blade assembly rotation moves the blade around the vertical drive axis",
        blade_center_rest is not None
        and blade_center_spun is not None
        and abs(blade_center_rest[0] - blade_center_spun[0]) > 0.012
        and abs(blade_center_rest[1] - blade_center_spun[1]) > 0.012,
        details=f"rest={blade_center_rest}, spun={blade_center_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
