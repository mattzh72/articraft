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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sound_enclosure_blender")

    base_black = model.material("base_black", rgba=(0.14, 0.15, 0.16, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.08, 0.09, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    clear_poly = model.material("clear_poly", rgba=(0.82, 0.90, 0.98, 0.22))
    smoke_poly = model.material("smoke_poly", rgba=(0.78, 0.86, 0.94, 0.18))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.24, 0.26, 0.136)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=base_black,
        name="base_shell",
    )
    base.visual(
        Box((0.210, 0.220, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=base_black,
        name="upper_shroud",
    )
    base.visual(
        Box((0.022, 0.206, 0.018)),
        origin=Origin(xyz=(-0.106, 0.0, 0.161)),
        material=base_black,
        name="rear_hinge_rail",
    )
    base.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
        material=stainless,
        name="drive_plinth",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=rubber_black,
        name="drive_coupling",
    )
    base.visual(
        Box((0.010, 0.120, 0.070)),
        origin=Origin(xyz=(0.108, 0.0, 0.100), rpy=(0.0, -0.52, 0.0)),
        material=fascia_black,
        name="control_panel",
    )
    base.visual(
        Box((0.002, 0.084, 0.050)),
        origin=Origin(xyz=(0.112, 0.0, 0.103), rpy=(0.0, -0.52, 0.0)),
        material=stainless,
        name="keypad_overlay",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.26, 0.19)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    jar = model.part("jar")
    jar.visual(
        Box((0.014, 0.116, 0.040)),
        origin=Origin(xyz=(0.050, 0.0, 0.020)),
        material=rubber_black,
        name="jar_collar_front",
    )
    jar.visual(
        Box((0.014, 0.116, 0.040)),
        origin=Origin(xyz=(-0.050, 0.0, 0.020)),
        material=rubber_black,
        name="jar_collar_back",
    )
    jar.visual(
        Box((0.086, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, 0.050, 0.020)),
        material=rubber_black,
        name="jar_collar_right",
    )
    jar.visual(
        Box((0.086, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, -0.050, 0.020)),
        material=rubber_black,
        name="jar_collar_left",
    )
    jar.visual(
        Box((0.008, 0.118, 0.212)),
        origin=Origin(xyz=(0.060, 0.0, 0.142), rpy=(0.0, 0.08, 0.0)),
        material=clear_poly,
        name="jar_front_wall",
    )
    jar.visual(
        Box((0.008, 0.118, 0.212)),
        origin=Origin(xyz=(-0.060, 0.0, 0.142), rpy=(0.0, -0.08, 0.0)),
        material=clear_poly,
        name="jar_back_wall",
    )
    jar.visual(
        Box((0.108, 0.008, 0.212)),
        origin=Origin(xyz=(0.0, 0.060, 0.142), rpy=(-0.08, 0.0, 0.0)),
        material=clear_poly,
        name="jar_right_wall",
    )
    jar.visual(
        Box((0.108, 0.008, 0.212)),
        origin=Origin(xyz=(0.0, -0.060, 0.142), rpy=(0.08, 0.0, 0.0)),
        material=clear_poly,
        name="jar_left_wall",
    )
    jar.visual(
        Box((0.014, 0.132, 0.012)),
        origin=Origin(xyz=(0.062, 0.0, 0.250)),
        material=stainless,
        name="jar_rim_front",
    )
    jar.visual(
        Box((0.014, 0.132, 0.012)),
        origin=Origin(xyz=(-0.062, 0.0, 0.250)),
        material=stainless,
        name="jar_rim_back",
    )
    jar.visual(
        Box((0.104, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.062, 0.250)),
        material=stainless,
        name="jar_rim_right",
    )
    jar.visual(
        Box((0.104, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.062, 0.250)),
        material=stainless,
        name="jar_rim_left",
    )
    jar.visual(
        Box((0.132, 0.132, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.259)),
        material=rubber_black,
        name="jar_lid",
    )
    jar.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=rubber_black,
        name="jar_cap",
    )
    jar.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(-0.004, 0.068, 0.205)),
        material=fascia_black,
        name="jar_handle_mount_upper",
    )
    jar.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(-0.004, 0.068, 0.095)),
        material=fascia_black,
        name="jar_handle_mount_lower",
    )
    jar_handle_geom = tube_from_spline_points(
        [
            (-0.004, 0.074, 0.214),
            (-0.010, 0.084, 0.205),
            (-0.014, 0.090, 0.162),
            (-0.012, 0.088, 0.116),
            (-0.006, 0.078, 0.092),
        ],
        radius=0.008,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )
    jar.visual(
        mesh_from_geometry(jar_handle_geom, "jar_handle"),
        material=fascia_black,
        name="jar_handle",
    )
    jar.inertial = Inertial.from_geometry(
        Box((0.16, 0.18, 0.29)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=blade_steel,
        name="blade_shaft",
    )
    blade_assembly.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=fascia_black,
        name="blade_hub",
    )
    blade_assembly.visual(
        Box((0.048, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.022), rpy=(0.0, 0.22, 0.0)),
        material=blade_steel,
        name="blade_span_a",
    )
    blade_assembly.visual(
        Box((0.048, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.023), rpy=(0.16, 0.0, math.pi / 2.0)),
        material=blade_steel,
        name="blade_span_b",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.040),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    hood = model.part("noise_hood")
    hood.visual(
        Box((0.008, 0.218, 0.314)),
        origin=Origin(xyz=(0.004, 0.0, 0.157)),
        material=smoke_poly,
        name="hood_back_wall",
    )
    hood.visual(
        Box((0.008, 0.218, 0.314)),
        origin=Origin(xyz=(0.222, 0.0, 0.157)),
        material=smoke_poly,
        name="hood_front_wall",
    )
    hood.visual(
        Box((0.230, 0.008, 0.314)),
        origin=Origin(xyz=(0.113, 0.105, 0.157)),
        material=smoke_poly,
        name="hood_right_wall",
    )
    hood.visual(
        Box((0.230, 0.008, 0.314)),
        origin=Origin(xyz=(0.113, -0.105, 0.157)),
        material=smoke_poly,
        name="hood_left_wall",
    )
    hood.visual(
        Box((0.230, 0.218, 0.008)),
        origin=Origin(xyz=(0.113, 0.0, 0.310)),
        material=smoke_poly,
        name="hood_top_panel",
    )
    hood.visual(
        Box((0.050, 0.024, 0.010)),
        origin=Origin(xyz=(0.226, 0.0, 0.140)),
        material=fascia_black,
        name="hood_pull",
    )
    hood.inertial = Inertial.from_geometry(
        Box((0.23, 0.22, 0.32)),
        mass=1.8,
        origin=Origin(xyz=(0.113, 0.0, 0.160)),
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.40,
            upper=0.40,
        ),
    )
    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=35.0),
    )
    model.articulation(
        "base_to_hood",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hood,
        origin=Origin(xyz=(-0.112, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
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

    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade_assembly = object_model.get_part("blade_assembly")
    hood = object_model.get_part("noise_hood")

    jar_lock = object_model.get_articulation("base_to_jar")
    blade_spin = object_model.get_articulation("base_to_blade")
    hood_hinge = object_model.get_articulation("base_to_hood")

    drive_plinth = base.get_visual("drive_plinth")
    jar_collar_front = jar.get_visual("jar_collar_front")
    jar_lid = jar.get_visual("jar_lid")
    jar_handle = jar.get_visual("jar_handle")
    hood_front_wall = hood.get_visual("hood_front_wall")
    hood_top_panel = hood.get_visual("hood_top_panel")

    ctx.expect_gap(
        jar,
        base,
        axis="z",
        positive_elem=jar_collar_front,
        negative_elem=drive_plinth,
        min_gap=-1.0e-6,
        max_gap=0.002,
        name="jar collar seats on drive plinth",
    )
    ctx.expect_within(
        jar,
        hood,
        axes="xy",
        margin=0.002,
        name="jar footprint stays inside closed hood",
    )
    ctx.expect_gap(
        hood,
        jar,
        axis="z",
        positive_elem=hood_top_panel,
        negative_elem=jar_lid,
        min_gap=0.030,
        name="closed hood roof clears jar lid",
    )
    ctx.expect_within(
        blade_assembly,
        jar,
        axes="xy",
        margin=0.040,
        name="blade assembly remains centered inside jar footprint",
    )
    ctx.check(
        "blade assembly uses continuous vertical spin",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(jar, elem=jar_handle)
    with ctx.pose({jar_lock: math.radians(20.0)}):
        twisted_handle_aabb = ctx.part_element_world_aabb(jar, elem=jar_handle)
        ctx.expect_gap(
            jar,
            base,
            axis="z",
            positive_elem=jar_collar_front,
            negative_elem=drive_plinth,
            min_gap=-1.0e-6,
            max_gap=0.002,
            name="jar stays seated while bayonet twisting",
        )
    rest_handle_center = _aabb_center(rest_handle_aabb)
    twisted_handle_center = _aabb_center(twisted_handle_aabb)
    ctx.check(
        "bayonet lock rotates jar handle around the drive",
        rest_handle_center is not None
        and twisted_handle_center is not None
        and twisted_handle_center[0] < rest_handle_center[0] - 0.015
        and abs(twisted_handle_center[1]) < abs(rest_handle_center[1]),
        details=f"rest={rest_handle_center}, twisted={twisted_handle_center}",
    )

    closed_front_aabb = ctx.part_element_world_aabb(hood, elem=hood_front_wall)
    with ctx.pose({hood_hinge: math.radians(72.0)}):
        open_front_aabb = ctx.part_element_world_aabb(hood, elem=hood_front_wall)
    closed_front_center = _aabb_center(closed_front_aabb)
    open_front_center = _aabb_center(open_front_aabb)
    ctx.check(
        "hood opens upward from the rear hinge",
        closed_front_center is not None
        and open_front_center is not None
        and open_front_center[2] > closed_front_center[2] + 0.090
        and open_front_center[0] < closed_front_center[0] - 0.060,
        details=f"closed={closed_front_center}, open={open_front_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
