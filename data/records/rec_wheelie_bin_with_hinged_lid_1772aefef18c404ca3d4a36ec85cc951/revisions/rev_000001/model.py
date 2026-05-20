from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    superellipse_profile,
)


RIM_Z = 0.34
HINGE_Y = -0.155
HINGE_Z = 0.365
WHEEL_Y = -0.125
WHEEL_Z = 0.044
WHEEL_X = 0.135
WHEEL_RADIUS = 0.039
WHEEL_WIDTH = 0.026
LID_OPEN = 2.05


def _loop(width: float, depth: float, z: float, *, y_center: float = 0.0, segments: int = 48):
    return [(x, y + y_center, z) for x, y in superellipse_profile(width, depth, exponent=3.0, segments=segments)]


def _add_loop(geom: MeshGeometry, points):
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _connect_loops(geom: MeshGeometry, a, b) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(a[i], b[i], b[j])
        geom.add_face(a[i], b[j], a[j])


def _cap_loop(geom: MeshGeometry, loop, point) -> None:
    center = geom.add_vertex(*point)
    count = len(loop)
    for i in range(count):
        geom.add_face(center, loop[i], loop[(i + 1) % count])


def _bin_shell_mesh() -> MeshGeometry:
    """Thin, open-topped tapered shell with an actual interior cavity and floor."""
    geom = MeshGeometry()
    outer_bottom = _add_loop(geom, _loop(0.170, 0.210, 0.035, y_center=0.020))
    outer_top = _add_loop(geom, _loop(0.242, 0.300, RIM_Z))
    inner_top = _add_loop(geom, _loop(0.210, 0.264, RIM_Z - 0.014))
    inner_floor = _add_loop(geom, _loop(0.146, 0.182, 0.064, y_center=0.020))

    _connect_loops(geom, outer_bottom, outer_top)
    _connect_loops(geom, outer_top, inner_top)
    _connect_loops(geom, inner_top, inner_floor)
    _cap_loop(geom, inner_floor, (0.0, 0.020, 0.064))
    _cap_loop(geom, outer_bottom, (0.0, 0.020, 0.035))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wheelie_bin")

    green_plastic = model.material("recycled_green_plastic", rgba=(0.07, 0.32, 0.18, 1.0))
    darker_green = model.material("darker_green_plastic", rgba=(0.04, 0.23, 0.13, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    axle_gray = model.material("galvanized_axle", rgba=(0.58, 0.60, 0.58, 1.0))
    rim_gray = model.material("molded_gray_rim", rgba=(0.33, 0.35, 0.34, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_geometry(_bin_shell_mesh(), "tapered_hollow_shell"),
        material=green_plastic,
        name="hollow_shell",
    )
    # Molded ribs are shallow and embedded into the tapered body so they read as
    # stiffening detail rather than floating ornament.
    shell.visual(Box((0.014, 0.026, 0.205)), origin=Origin(xyz=(-0.050, 0.145, 0.190)), material=darker_green, name="front_rib_0")
    shell.visual(Box((0.014, 0.026, 0.205)), origin=Origin(xyz=(0.050, 0.145, 0.190)), material=darker_green, name="front_rib_1")
    shell.visual(Box((0.140, 0.022, 0.018)), origin=Origin(xyz=(0.0, 0.145, 0.090)), material=darker_green, name="toe_rib")
    shell.visual(Box((0.050, 0.070, 0.044)), origin=Origin(xyz=(-0.066, 0.110, 0.022)), material=darker_green, name="front_foot_0")
    shell.visual(Box((0.050, 0.070, 0.044)), origin=Origin(xyz=(0.066, 0.110, 0.022)), material=darker_green, name="front_foot_1")

    shell.visual(
        Cylinder(radius=0.0055, length=0.302),
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="rear_axle",
    )
    shell.visual(Box((0.030, 0.080, 0.052)), origin=Origin(xyz=(-0.107, -0.096, 0.058)), material=darker_green, name="axle_boss_0")
    shell.visual(Box((0.030, 0.080, 0.052)), origin=Origin(xyz=(0.107, -0.096, 0.058)), material=darker_green, name="axle_boss_1")
    shell.visual(Box((0.230, 0.070, 0.056)), origin=Origin(xyz=(0.0, -0.065, 0.058)), material=darker_green, name="axle_bridge")

    # A low-profile rear grip and hinge ledge keep the desktop version compact.
    shell.visual(
        Cylinder(radius=0.006, length=0.198),
        origin=Origin(xyz=(0.0, -0.171, 0.307), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_green,
        name="rear_grip",
    )
    shell.visual(Box((0.014, 0.036, 0.070)), origin=Origin(xyz=(-0.105, -0.158, 0.300)), material=darker_green, name="grip_stem_0")
    shell.visual(Box((0.014, 0.036, 0.070)), origin=Origin(xyz=(0.105, -0.158, 0.300)), material=darker_green, name="grip_stem_1")
    shell.visual(Box((0.230, 0.024, 0.014)), origin=Origin(xyz=(0.0, -0.155, 0.334)), material=darker_green, name="hinge_ledge")
    shell.visual(Box((0.030, 0.020, 0.016)), origin=Origin(xyz=(-0.095, -0.172, 0.3465)), material=darker_green, name="hinge_seat_0")
    shell.visual(Box((0.030, 0.020, 0.016)), origin=Origin(xyz=(0.095, -0.172, 0.3465)), material=darker_green, name="hinge_seat_1")

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            # A shallow rounded slab spanning the full rim; local +Y extends
            # from the hinge line toward the front lip.
            ExtrudeGeometry(
                superellipse_profile(0.262, 0.314, exponent=3.2, segments=48),
                0.018,
                cap=True,
                center=True,
            ),
            "lid_rounded_panel",
        ),
        origin=Origin(xyz=(0.0, 0.147, -0.010)),
        material=green_plastic,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.246),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_green,
        name="hinge_barrel",
    )
    lid.visual(Box((0.245, 0.020, 0.012)), origin=Origin(xyz=(0.0, 0.298, -0.006)), material=darker_green, name="front_lip")
    lid.visual(Box((0.012, 0.225, 0.008)), origin=Origin(xyz=(-0.058, 0.145, 0.003)), material=darker_green, name="lid_rib_0")
    lid.visual(Box((0.012, 0.225, 0.008)), origin=Origin(xyz=(0.058, 0.145, 0.003)), material=darker_green, name="lid_rib_1")

    def add_wheel(name: str) -> None:
        wheel = model.part(name)
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    WHEEL_RADIUS,
                    WHEEL_WIDTH,
                    inner_radius=0.026,
                    tread=TireTread(style="circumferential", depth=0.0025, count=3),
                    grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
                    sidewall=TireSidewall(style="rounded", bulge=0.04),
                ),
                f"{name}_tire",
            ),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.026,
                    0.020,
                    rim=WheelRim(inner_radius=0.018, flange_height=0.003, flange_thickness=0.0015),
                    hub=WheelHub(radius=0.010, width=0.024, cap_style="flat"),
                    face=WheelFace(dish_depth=0.0015, front_inset=0.001),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.0015, window_radius=0.003),
                    bore=WheelBore(style="round", diameter=0.014),
                ),
                f"{name}_rim",
            ),
            material=rim_gray,
            name="rim",
        )

    add_wheel("wheel_0")
    add_wheel("wheel_1")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=LID_OPEN),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child="wheel_0",
        origin=Origin(xyz=(-WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=18.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child="wheel_1",
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    ctx.check("functional_parts_present", all((shell, lid, wheel_0, wheel_1)), "Expected shell, lid, and two wheels.")
    ctx.check("lid_hinge_revolute", getattr(lid_hinge, "articulation_type", None) == ArticulationType.REVOLUTE, "Lid must use a revolute hinge.")
    ctx.check(
        "wheel_joints_continuous",
        all(getattr(j, "articulation_type", None) == ArticulationType.CONTINUOUS for j in (wheel_0_spin, wheel_1_spin)),
        "Both rear wheels should roll on continuous spin joints.",
    )

    if shell and lid:
        with ctx.pose({lid_hinge: 0.0}):
            ctx.expect_gap(
                lid,
                shell,
                axis="z",
                min_gap=0.003,
                max_gap=0.030,
                positive_elem="lid_panel",
                negative_elem="hollow_shell",
                name="closed lid clears top rim",
            )
            ctx.expect_overlap(lid, shell, axes="xy", min_overlap=0.16, elem_a="lid_panel", elem_b="hollow_shell", name="lid covers bin opening")

        closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: LID_OPEN}):
            open_aabb = ctx.part_world_aabb(lid)
            ctx.expect_gap(lid, shell, axis="z", min_gap=0.010, positive_elem="front_lip", negative_elem="hinge_ledge", name="open lid clears rear ledge")
        ctx.check(
            "lid_swings_up_and_back",
            closed_aabb is not None
            and open_aabb is not None
            and float(open_aabb[1][2]) > float(closed_aabb[1][2]) + 0.10
            and float(open_aabb[0][1]) < float(closed_aabb[0][1]) - 0.035,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    if shell and wheel_0 and wheel_1:
        ctx.expect_overlap(wheel_0, shell, axes="yz", elem_a="rim", elem_b="rear_axle", min_overlap=0.010, name="wheel_0 captured on axle line")
        ctx.expect_overlap(wheel_1, shell, axes="yz", elem_a="rim", elem_b="rear_axle", min_overlap=0.010, name="wheel_1 captured on axle line")
        ctx.expect_origin_gap(wheel_1, wheel_0, axis="x", min_gap=0.26, max_gap=0.28, name="wheels flank compact shell")

    return ctx.report()


object_model = build_object_model()
