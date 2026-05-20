from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_bar_stool")

    chrome = _mat(model, "polished_chrome", (0.78, 0.80, 0.82, 1.0))
    dark_chrome = _mat(model, "shadow_chrome", (0.33, 0.35, 0.36, 1.0))
    black_vinyl = _mat(model, "black_vinyl", (0.025, 0.023, 0.022, 1.0))
    black_plastic = _mat(model, "black_plastic", (0.04, 0.04, 0.045, 1.0))
    grey_plate = _mat(model, "brushed_underplate", (0.38, 0.39, 0.40, 1.0))

    base = model.part("base")

    base.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.235, 0.000),
                    (0.278, 0.010),
                    (0.288, 0.025),
                    (0.250, 0.040),
                    (0.090, 0.050),
                    (0.058, 0.064),
                ],
                [(0.040, 0.006), (0.040, 0.064)],
                segments=96,
                start_cap="flat",
                end_cap="flat",
            ),
            "domed_round_base",
        ),
        material=chrome,
        name="domed_round_base",
    )
    base.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.094, 0.043), (0.094, 0.077)],
                [(0.038, 0.043), (0.038, 0.077)],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "lower_collar",
        ),
        material=dark_chrome,
        name="lower_collar",
    )
    base.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.057, 0.056), (0.060, 0.080), (0.052, 0.480)],
                [(0.037, 0.058), (0.037, 0.480)],
                segments=80,
                start_cap="flat",
                end_cap="flat",
            ),
            "outer_sleeve",
        ),
        material=chrome,
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.070, 0.465), (0.070, 0.495)],
                [(0.038, 0.465), (0.038, 0.495)],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "sleeve_lip",
        ),
        material=dark_chrome,
        name="sleeve_lip",
    )
    for z_center, label in ((0.112, "lower"), (0.452, "upper")):
        base.visual(
            Box((0.026, 0.018, 0.050)),
            origin=Origin(xyz=(0.044, 0.0, z_center)),
            material=black_plastic,
            name=f"{label}_guide_pad_0",
        )
        base.visual(
            Box((0.026, 0.018, 0.050)),
            origin=Origin(xyz=(-0.044, 0.0, z_center)),
            material=black_plastic,
            name=f"{label}_guide_pad_1",
        )
        base.visual(
            Box((0.018, 0.026, 0.050)),
            origin=Origin(xyz=(0.0, 0.044, z_center)),
            material=black_plastic,
            name=f"{label}_guide_pad_2",
        )
        base.visual(
            Box((0.018, 0.026, 0.050)),
            origin=Origin(xyz=(0.0, -0.044, z_center)),
            material=black_plastic,
            name=f"{label}_guide_pad_3",
        )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.031, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=chrome,
        name="inner_mast",
    )
    lift_column.visual(
        Cylinder(radius=0.050, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        material=dark_chrome,
        name="top_boss",
    )
    lift_column.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
        material=chrome,
        name="bearing_disk",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.380, 0.380, 0.055, corner_segments=10),
                0.075,
                center=True,
            ),
            "rounded_square_cushion",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=black_vinyl,
        name="rounded_square_cushion",
    )
    seat.visual(
        Box((0.280, 0.280, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=grey_plate,
        name="under_plate",
    )
    seat.visual(
        Box((0.315, 0.315, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=black_vinyl,
        name="cushion_underboard",
    )
    seat.visual(
        Cylinder(radius=0.066, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=chrome,
        name="swivel_socket",
    )
    seat.visual(
        Box((0.075, 0.130, 0.014)),
        origin=Origin(xyz=(0.153, 0.0, 0.058)),
        material=grey_plate,
        name="lever_mount_web",
    )
    seat.visual(
        Box((0.045, 0.012, 0.050)),
        origin=Origin(xyz=(0.192, 0.035, 0.032)),
        material=grey_plate,
        name="clevis_ear_0",
    )
    seat.visual(
        Box((0.045, 0.012, 0.050)),
        origin=Origin(xyz=(0.192, -0.035, 0.032)),
        material=grey_plate,
        name="clevis_ear_1",
    )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.012, length=0.058),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="pivot_barrel",
    )
    release_lever.visual(
        Cylinder(radius=0.0085, length=0.145),
        origin=Origin(xyz=(0.071, 0.0, -0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="lever_rod",
    )
    release_lever.visual(
        Box((0.055, 0.026, 0.018)),
        origin=Origin(xyz=(0.155, 0.0, -0.018)),
        material=black_plastic,
        name="finger_tab",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.22, lower=0.0, upper=0.255),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.5),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=release_lever,
        origin=Origin(xyz=(0.192, 0.0, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=5.0, lower=-0.42, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    seat = object_model.get_part("seat")
    release_lever = object_model.get_part("release_lever")
    height_slide = object_model.get_articulation("height_slide")
    seat_swivel = object_model.get_articulation("seat_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="inner mast is centered inside sleeve",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.34,
        name="collapsed mast remains deeply inserted",
    )
    with ctx.pose({height_slide: 0.255}):
        ctx.expect_within(
            lift_column,
            base,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="raised mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.17,
            name="raised mast retains insertion",
        )

    ctx.expect_contact(
        seat,
        lift_column,
        elem_a="swivel_socket",
        elem_b="bearing_disk",
        contact_tol=0.002,
        name="seat swivel socket rests on bearing disk",
    )
    ctx.expect_within(
        release_lever,
        seat,
        axes="xz",
        inner_elem="pivot_barrel",
        outer_elem="clevis_ear_0",
        margin=0.010,
        name="lever pivot barrel sits under the seat mount",
    )

    tab_rest = ctx.part_element_world_aabb(release_lever, elem="finger_tab")
    with ctx.pose({lever_pivot: 0.45}):
        tab_down = ctx.part_element_world_aabb(release_lever, elem="finger_tab")
    ctx.check(
        "release lever rotates downward",
        tab_rest is not None and tab_down is not None and tab_down[0][2] < tab_rest[0][2] - 0.035,
        details=f"rest={tab_rest}, pulled={tab_down}",
    )

    seat_rest = ctx.part_world_position(seat)
    with ctx.pose({seat_swivel: pi}):
        seat_rotated = ctx.part_world_position(seat)
    ctx.check(
        "seat swivels about same vertical axis",
        seat_rest is not None
        and seat_rotated is not None
        and abs(seat_rotated[0] - seat_rest[0]) < 1e-6
        and abs(seat_rotated[1] - seat_rest[1]) < 1e-6,
        details=f"rest={seat_rest}, rotated={seat_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
