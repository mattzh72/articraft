from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_pitch_cradle")

    steel = model.material("dark_powder_coated_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    edge_steel = model.material("worn_steel_edges", rgba=(0.50, 0.52, 0.52, 1.0))
    bronze = model.material("oiled_bronze_bushings", rgba=(0.74, 0.50, 0.22, 1.0))
    panel_blue = model.material("satin_blue_faceplate", rgba=(0.05, 0.18, 0.34, 1.0))
    black = model.material("black_recess", rgba=(0.006, 0.006, 0.007, 1.0))

    fork_frame = model.part("fork_frame")
    fork_frame.visual(
        Box((0.46, 0.25, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=steel,
        name="floor_plate",
    )
    fork_yoke = TrunnionYokeGeometry(
        (0.36, 0.18, 0.20),
        span_width=0.240,
        trunnion_diameter=0.048,
        trunnion_center_z=0.105,
        base_thickness=0.026,
        corner_radius=0.008,
        center=False,
    )
    fork_frame.visual(
        mesh_from_geometry(fork_yoke, "fork_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=steel,
        name="fork_yoke",
    )

    bushing_mesh = mesh_from_geometry(
        # The bronze rings are modeled with a hole just under the shaft radius,
        # giving a tiny bearing interference/contact so the moving plate is
        # physically supported by the grounded fork.
        TorusGeometry(0.023, 0.008, radial_segments=20, tubular_segments=32),
        "bushing_ring",
    )
    for index, x in enumerate((-0.183, 0.183)):
        fork_frame.visual(
            bushing_mesh,
            origin=Origin(xyz=(x, 0.0, 0.121), rpy=(0.0, pi / 2.0, 0.0)),
            material=bronze,
            name=f"bushing_{index}",
        )
    for index, (x, y) in enumerate(
        (
            (-0.170, -0.095),
            (0.170, -0.095),
            (-0.170, 0.095),
            (0.170, 0.095),
        )
    ):
        fork_frame.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.019)),
            material=edge_steel,
            name=f"base_bolt_{index}",
        )

    faceplate = model.part("faceplate")
    face_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.180, 0.150, 0.015, corner_segments=8),
            0.018,
            cap=True,
            center=True,
        ),
        "face_panel",
    )
    faceplate.visual(
        face_panel_mesh,
        # The panel is a rounded rectangular plate in the local XZ plane.  Its
        # lower rounded edge overlaps the shaft hub slightly, like a welded tab.
        origin=Origin(xyz=(0.0, 0.0, 0.086), rpy=(pi / 2.0, 0.0, 0.0)),
        material=panel_blue,
        name="face_panel",
    )
    faceplate.visual(
        Cylinder(radius=0.016, length=0.382),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=edge_steel,
        name="cross_shaft",
    )
    faceplate.visual(
        Cylinder(radius=0.026, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, pi / 2.0, 0.0)),
        material=edge_steel,
        name="root_hub",
    )
    faceplate.visual(
        Box((0.130, 0.004, 0.082)),
        origin=Origin(xyz=(0.0, -0.010, 0.092)),
        material=black,
        name="front_inset",
    )
    for index, (x, z) in enumerate(
        ((-0.069, 0.043), (0.069, 0.043), (-0.069, 0.141), (0.069, 0.141))
    ):
        faceplate.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x, -0.011, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=edge_steel,
            name=f"face_screw_{index}",
        )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=fork_frame,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork_frame = object_model.get_part("fork_frame")
    faceplate = object_model.get_part("faceplate")
    pitch_axis = object_model.get_articulation("pitch_axis")

    for bushing_name in ("bushing_0", "bushing_1"):
        ctx.allow_overlap(
            faceplate,
            fork_frame,
            elem_a="cross_shaft",
            elem_b=bushing_name,
            reason=(
                "The shaft is intentionally modeled with a tiny seated "
                "interference in the bronze bearing ring so the pitch cradle "
                "is supported rather than floating."
            ),
        )

    ctx.check(
        "single fork-supported pitch joint",
        len(object_model.articulations) == 1
        and pitch_axis.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_within(
        faceplate,
        fork_frame,
        axes="x",
        inner_elem="face_panel",
        outer_elem="fork_yoke",
        margin=0.0,
        name="faceplate sits between the fork arms",
    )
    ctx.expect_overlap(
        faceplate,
        fork_frame,
        axes="x",
        elem_a="cross_shaft",
        elem_b="fork_yoke",
        min_overlap=0.32,
        name="cross shaft spans the fork supports",
    )
    for bushing_name in ("bushing_0", "bushing_1"):
        ctx.expect_overlap(
            faceplate,
            fork_frame,
            axes="xyz",
            elem_a="cross_shaft",
            elem_b=bushing_name,
            min_overlap=0.008,
            name=f"cross shaft is captured by {bushing_name}",
        )

    rest_aabb = ctx.part_element_world_aabb(faceplate, elem="face_panel")
    with ctx.pose({pitch_axis: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(faceplate, elem="face_panel")

    rest_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    tilted_y = None if tilted_aabb is None else (tilted_aabb[0][1] + tilted_aabb[1][1]) / 2.0
    ctx.check(
        "positive pitch tilts the faceplate about the shaft",
        rest_y is not None and tilted_y is not None and tilted_y < rest_y - 0.025,
        details=f"rest_y={rest_y}, tilted_y={tilted_y}",
    )

    return ctx.report()


object_model = build_object_model()
