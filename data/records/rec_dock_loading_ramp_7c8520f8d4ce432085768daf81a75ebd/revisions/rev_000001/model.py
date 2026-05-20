from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(length: float, outer_radius: float, inner_radius: float, *, axis: str, segments: int = 32) -> MeshGeometry:
    """Closed hollow tube mesh centered at the local origin."""
    geom = MeshGeometry()

    def point(along: float, radius: float, angle: float) -> tuple[float, float, float]:
        c = math.cos(angle) * radius
        s = math.sin(angle) * radius
        if axis == "x":
            return (along, c, s)
        if axis == "y":
            return (c, along, s)
        return (c, s, along)

    half = length * 0.5
    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        outer0.append(geom.add_vertex(*point(-half, outer_radius, angle)))
        outer1.append(geom.add_vertex(*point(half, outer_radius, angle)))
        inner0.append(geom.add_vertex(*point(-half, inner_radius, angle)))
        inner1.append(geom.add_vertex(*point(half, inner_radius, angle)))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # Inner bore, with winding reversed.
        geom.add_face(inner0[j], inner0[i], inner1[i])
        geom.add_face(inner0[j], inner1[i], inner1[j])
        # Annular end caps.
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pivoting_dock_board")

    dark_steel = model.material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    worn_steel = model.material("ribbed_galvanized_steel", rgba=(0.47, 0.50, 0.50, 1.0))
    hinge_steel = model.material("polished_hinge_pin", rgba=(0.68, 0.70, 0.68, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    black_rubber = model.material("black_warning_stripe", rgba=(0.02, 0.02, 0.018, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.42, 0.40, 0.36, 1.0))

    dock_z = 1.10
    deck_length = 1.80
    deck_width = 1.50
    deck_thick = 0.065
    deck_rear_x = 0.06
    deck_top_z = -0.018
    side_hinge_x0 = 0.12
    side_hinge_len = 1.58
    side_hinge_y = deck_width * 0.5 + 0.030
    side_hinge_z = 0.020
    curb_height = 0.25
    curb_thick = 0.040

    dock_frame = model.part("dock_frame")
    dock_frame.visual(
        Box((0.10, 2.06, dock_z)),
        origin=Origin(xyz=(-0.105, 0.0, dock_z * 0.5)),
        material=concrete,
        name="dock_face",
    )
    dock_frame.visual(
        Box((0.62, 2.10, 0.08)),
        origin=Origin(xyz=(-0.405, 0.0, dock_z + deck_top_z - 0.040)),
        material=concrete,
        name="dock_floor_slab",
    )
    dock_frame.visual(
        Box((0.045, 1.86, 0.13)),
        origin=Origin(xyz=(-0.085, 0.0, dock_z - 0.030)),
        material=dark_steel,
        name="face_frame_header",
    )
    dock_frame.visual(
        Cylinder(radius=0.022, length=deck_width + 0.24),
        origin=Origin(xyz=(0.0, 0.0, dock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="rear_hinge_pin",
    )
    for idx, y in enumerate((-0.84, -0.31, 0.31, 0.84)):
        dock_frame.visual(
            Box((0.095, 0.105, 0.125)),
            origin=Origin(xyz=(-0.030, y, dock_z - 0.015)),
            material=dark_steel,
            name=f"frame_hinge_lug_{idx}",
        )

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thick)),
        origin=Origin(xyz=(deck_rear_x + deck_length * 0.5, 0.0, deck_top_z - deck_thick * 0.5)),
        material=worn_steel,
        name="deck_plate",
    )
    deck.visual(
        Box((0.045, deck_width, 0.055)),
        origin=Origin(xyz=(0.048, 0.0, -0.028)),
        material=worn_steel,
        name="rear_cleat_bar",
    )
    deck.visual(
        Box((0.240, deck_width - 0.10, 0.026)),
        origin=Origin(xyz=(deck_rear_x + deck_length + 0.115, 0.0, deck_top_z - 0.053)),
        material=worn_steel,
        name="front_lip",
    )
    # Raised transverse anti-slip ribs on the otherwise flat steel deck.
    for idx in range(9):
        x = deck_rear_x + 0.22 + idx * 0.17
        deck.visual(
            Box((0.030, deck_width - 0.22, 0.012)),
            origin=Origin(xyz=(x, 0.0, deck_top_z + 0.006)),
            material=worn_steel,
            name=f"deck_rib_{idx}",
        )
    # Rear cleated edge blocks immediately in front of the hinge tube.
    for idx, y in enumerate((-0.60, -0.40, -0.20, 0.0, 0.20, 0.40, 0.60)):
        deck.visual(
            Box((0.075, 0.105, 0.026)),
            origin=Origin(xyz=(0.125, y, deck_top_z + 0.013)),
            material=worn_steel,
            name=f"rear_cleat_{idx}",
        )
    for idx, (y_center, seg_len) in enumerate(((-0.565, 0.250), (0.0, 0.360), (0.565, 0.250))):
        deck.visual(
            mesh_from_geometry(_tube_mesh(seg_len, 0.044, 0.022, axis="y"), f"rear_hinge_barrel_{idx}"),
            origin=Origin(xyz=(0.0, y_center, 0.0)),
            material=worn_steel,
            name=f"rear_hinge_barrel_{idx}",
        )
    for side_idx, side in enumerate((-1.0, 1.0)):
        deck.visual(
            Cylinder(radius=0.014, length=side_hinge_len),
            origin=Origin(
                xyz=(side_hinge_x0 + side_hinge_len * 0.5, side * side_hinge_y, side_hinge_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_steel,
            name=f"side_hinge_pin_{side_idx}",
        )
        for lug_idx, x in enumerate((side_hinge_x0 + 0.42, side_hinge_x0 + 0.96, side_hinge_x0 + 1.46)):
            deck.visual(
                Box((0.070, 0.030, 0.056)),
                origin=Origin(xyz=(x, side * (deck_width * 0.5 + 0.012), deck_top_z + 0.026)),
                material=dark_steel,
                name=f"curb_mount_{side_idx}_{lug_idx}",
            )

    for side_idx, side in enumerate((-1.0, 1.0)):
        curb = model.part(f"side_curb_{side_idx}")
        curb.visual(
            Box((side_hinge_len, curb_thick, curb_height)),
            origin=Origin(xyz=(side_hinge_len * 0.5, side * 0.040, curb_height * 0.5)),
            material=safety_yellow,
            name="curb_panel",
        )
        curb.visual(
            Box((side_hinge_len, 0.014, 0.035)),
            origin=Origin(xyz=(side_hinge_len * 0.5, side * 0.067, curb_height * 0.55)),
            material=black_rubber,
            name="warning_stripe",
        )
        for seg_idx, x in enumerate((0.16, 0.73, 1.27)):
            curb.visual(
                mesh_from_geometry(_tube_mesh(0.260, 0.026, 0.014, axis="x"), f"curb_hinge_barrel_{side_idx}_{seg_idx}"),
                origin=Origin(xyz=(x, 0.0, 0.0)),
                material=safety_yellow,
                name=f"curb_hinge_barrel_{seg_idx}",
            )
        model.articulation(
            f"deck_to_curb_{side_idx}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=curb,
            origin=Origin(xyz=(side_hinge_x0, side * side_hinge_y, side_hinge_z)),
            axis=(-side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=math.pi / 2.0),
        )

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=dock_frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, dock_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.45, lower=-0.15, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock_frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("deck")
    deck_hinge = object_model.get_articulation("frame_to_deck")

    # The visible pins are intentionally represented as solid captured shafts
    # inside hinge-barrel proxy meshes.
    for idx, min_len in enumerate((0.20, 0.30, 0.20)):
        barrel = f"rear_hinge_barrel_{idx}"
        ctx.allow_overlap(
            deck,
            dock_frame,
            elem_a=barrel,
            elem_b="rear_hinge_pin",
            reason="The rear hinge pin is intentionally captured through the deck hinge barrel.",
        )
        ctx.expect_within(
            dock_frame,
            deck,
            axes="xz",
            inner_elem="rear_hinge_pin",
            outer_elem=barrel,
            margin=0.001,
            name=f"rear pin sits inside barrel {idx}",
        )
        ctx.expect_overlap(
            deck,
            dock_frame,
            axes="y",
            elem_a=barrel,
            elem_b="rear_hinge_pin",
            min_overlap=min_len,
            name=f"rear barrel {idx} remains on hinge pin",
        )

    for side_idx in (0, 1):
        curb = object_model.get_part(f"side_curb_{side_idx}")
        for seg_idx in (0, 1, 2):
            barrel = f"curb_hinge_barrel_{seg_idx}"
            pin = f"side_hinge_pin_{side_idx}"
            ctx.allow_overlap(
                deck,
                curb,
                elem_a=pin,
                elem_b=barrel,
                reason="The curb hinge barrel is intentionally captured around the deck side hinge pin.",
            )
            ctx.expect_within(
                deck,
                curb,
                axes="yz",
                inner_elem=pin,
                outer_elem=barrel,
                margin=0.001,
                name=f"curb {side_idx} pin sits inside barrel {seg_idx}",
            )
            ctx.expect_overlap(
                deck,
                curb,
                axes="x",
                elem_a=pin,
                elem_b=barrel,
                min_overlap=0.20,
                name=f"curb {side_idx} barrel {seg_idx} remains on pin",
            )

    ctx.expect_overlap(
        deck,
        dock_frame,
        axes="y",
        elem_a="deck_plate",
        elem_b="dock_face",
        min_overlap=1.40,
        name="deck spans the dock opening width",
    )

    rest_lip = ctx.part_element_world_aabb(deck, elem="front_lip")
    with ctx.pose({deck_hinge: 0.30}):
        lowered_lip = ctx.part_element_world_aabb(deck, elem="front_lip")
    ctx.check(
        "deck hinge lowers the bridge lip",
        rest_lip is not None and lowered_lip is not None and lowered_lip[0][2] < rest_lip[0][2] - 0.15,
        details=f"rest_lip={rest_lip}, lowered_lip={lowered_lip}",
    )

    for side_idx, side in ((0, -1.0), (1, 1.0)):
        curb = object_model.get_part(f"side_curb_{side_idx}")
        curb_hinge = object_model.get_articulation(f"deck_to_curb_{side_idx}")
        ctx.expect_overlap(
            curb,
            deck,
            axes="x",
            elem_a="curb_panel",
            elem_b="deck_plate",
            min_overlap=1.45,
            name=f"curb {side_idx} runs along deck edge",
        )
        upright_panel = ctx.part_element_world_aabb(curb, elem="curb_panel")
        with ctx.pose({curb_hinge: math.pi / 2.0}):
            folded_panel = ctx.part_element_world_aabb(curb, elem="curb_panel")
        if upright_panel is not None and folded_panel is not None:
            upright_y_center = (upright_panel[0][1] + upright_panel[1][1]) * 0.5
            folded_y_center = (folded_panel[0][1] + folded_panel[1][1]) * 0.5
            upright_z_center = (upright_panel[0][2] + upright_panel[1][2]) * 0.5
            folded_z_center = (folded_panel[0][2] + folded_panel[1][2]) * 0.5
            ok = side * folded_y_center > side * upright_y_center + 0.08 and folded_z_center < upright_z_center - 0.08
        else:
            ok = False
        ctx.check(
            f"curb {side_idx} folds outward and down",
            ok,
            details=f"upright_panel={upright_panel}, folded_panel={folded_panel}",
        )

    return ctx.report()


object_model = build_object_model()
